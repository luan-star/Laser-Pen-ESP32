# ĐỒ ÁN TỐT NGHIỆP: HỆ THỐNG BÚT THÔNG MINH (SMART PEN)

* **Sinh viên thực hiện:** Nguyễn Hữu Luân (MSSV: 20200253)
* **Giáo viên hướng dẫn:** ThS. Đỗ Quốc Minh Đăng
* **Bộ môn/Khoa:** Điện tử - Viễn thông, Trường Đại học Khoa học Tự nhiên - ĐHQG HCM

Kho lưu trữ này chứa toàn bộ mã nguồn kiểm soát phần mềm nhúng (Firmware) phân tán cho hệ thống Bút thông minh. Hệ thống bao gồm 3 module vi điều khiển xử lý phối hợp đồng bộ thông qua các giao thức truyền thông không dây và có dây: UART, ESP-NOW và Bluetooth HID (BLE Mouse).

---

## 🗺️ Sơ đồ Kiến trúc Hệ thống tổng quát

```text
       ┌───────────────────────────┐
       │   ESP32 Bút Phát (TX)     │
       └─────────────┬─────────────┘
                     │
                     │ Giao thức ESP-NOW 
                     │ (Sự kiện nút bấm, cuộn, trạng thái nguồn)
                     ▼
       ┌───────────────────────────┐   Mạch UART (Tọa độ x, y)   ┌───────────────────────────┐
       │ ESP32-S3 Trạm Trung Tâm   │<───────────────────────────>│    ESP32-CAM Xử lý ảnh    │
       └─────────────┬─────────────┘                             │  (Thuật toán Bounded ROI) │
                     │                                           └───────────────────────────┘
                     │ Giao thức Bluetooth HID 
                     │ (Chuột BLE Absolute / Relative)
                     ▼
               ┌───────────┐
               │ Máy Tính  │
               └───────────┘
```

---

## 🧠 Phân tích chi tiết các Thuật toán cốt lõi

### 1. Module `ESP32CAM_Vision`: Thuật toán Bounded ROI & Dự đoán Quỹ đạo Động

Module này chịu trách nhiệm thu thập khung hình Grayscale ở độ phân giải QVGA ($320 \times 240$) từ cảm biến ảnh OV2640 với tốc độ cao, tiến hành lọc nhiễu số và định vị chính xác tọa độ điểm sáng laser dưới pixel (sub-pixel).

#### A. Giải thuật Khóa vùng quét khi mất dấu (Bounded ROI - Cập nhật mới)
* **Vấn đề hệ thống cũ:** Khi người dùng di chuyển bút quá nhanh hoặc đưa bút ra ngoài không gian tương tác tạm thời, hệ thống bị mất dấu bám vết (`has_tracking = false`). Ở phiên bản cũ, camera bắt buộc phải quét lại toàn bộ ảnh kích thước $320 \times 240 = 76.800$ điểm ảnh để tìm lại tia laser. Quá trình xử lý toàn khung hình này tiêu tốn rất nhiều chu kỳ CPU, gây sụt giảm FPS đột ngột và tạo độ trễ lớn (Latency) khi bắt lại tia.
* **Giải pháp cải tiến:** Thuật toán tận dụng dữ liệu ranh giới không gian làm việc thực tế (`Bounding Box`) thu được từ bước Hiệu chuẩn (Calibration) lưu trữ tại bộ nhớ NVS của ESP32-S3 và gửi qua UART. 
  * Biên độ vùng quét tối đa được giới hạn cứng bởi tọa độ hiệu chuẩn: `[bound_min_x, bound_max_x, bound_min_y, bound_max_y]`.
  * Để đảm bảo tính công thái học và an toàn biên, thuật toán tự động mở rộng vùng quét này thêm một biên độ dung sai dung hòa là **+10 pixel** về mỗi phía:

$$
\begin{cases} 
s_x = \max(0, x_{\min} - 10) \\ 
e_x = \min(W_{\text{frame}} - 1, x_{\max} + 10) 
\end{cases}
$$

  * Khi mất dấu, camera **chỉ quét tìm kiếm điểm sáng bên trong vùng ranh giới thu hẹp** này. Diện tích tính toán giảm xuống từ $50\%$ đến $70\%$, cho phép tốc độ hồi phục trạng thái bám vết (Recovery speed) diễn ra gần như tức thì với tốc độ khung hình (FPS) ổn định tối đa.

#### B. Thuật toán ROI Động dựa trên Kinematic (Động học Vật thể)
* Khi hệ thống đang ở trạng thái bám vết ổn định (`has_tracking = true`), vùng quét (ROI) thu hẹp lại thành một ô cửa sổ vuông siêu nhỏ xung quanh điểm Laser hiện tại để tối ưu thời gian xử lý.
* Véctơ vận tốc được tính toán theo thời gian thực giữa 2 khung hình liên tiếp $t-1$ và $t-2$:

$$
v_x = x_{t-1} - x_{t-2}, \quad v_y = y_{t-1} - y_{t-2}
$$

* Bán kính vùng quét động tự điều chỉnh tuyến tính theo độ lớn vận tốc di chuyển nhằm bắt kịp chuyển động rê nhanh của người dùng, kết hợp hệ số giảm chấn `damping = 0.8f` để bù trễ phần cứng:

$$
R_{\text{dynamic}} = \text{clamp}\left(15 + v_{\text{elocity}} \times \frac{v_{\text{factor}}}{10}, 15, 100\right)
$$

$$
x_{\text{predict}} = x_{t-1} + v_x \times \text{damping}
$$

#### C. Thuật toán định vị Trọng tâm dưới điểm ảnh (Sub-pixel Centroid)
* Sau khi xác định được tọa độ pixel sáng nhất (Peak Detection) vượt ngưỡng `bright_threshold`, một vùng cửa sổ phụ kích thước $31 \times 31$ (`SEARCH_RADIUS = 15`) được thiết lập độc lập.
* Thuật toán tính toán trọng tâm hình học dựa trên phân bố cường độ sáng (Center of Mass) để đạt độ chính xác dưới mức pixel, loại bỏ nhiễu răng cưa:

$$
X_{\text{out}} = \frac{\sum (x \cdot I_{(x,y)})}{\sum I_{(x,y)}}, \quad Y_{\text{out}} = \frac{\sum (y \cdot I_{(x,y)})}{\sum I_{(x,y)}} \quad (\text{với } I_{(x,y)} > T_{\text{bright}})
$$

---

### 2. Module `ESP32S3_Central`: Biến đổi Không gian & Nội suy Tọa độ Chuột

Module đóng vai trò bộ não trung tâm, điều phối toàn bộ vòng lặp dữ liệu lớn, tính toán ma trận hình học không gian phẳng và giao tiếp lớp HID Bluetooth.

#### A. Thuật toán Biến đổi Homography phẳng (Chuyển đổi Hệ tọa độ)
* Máy ảnh đặt ở một góc bất kỳ so với màn hình chiếu sẽ gây ra hiện tượng méo hình thang (Perspective Distortion). Thuật toán Homography thiết lập ma trận biến đổi tuần hoàn $3 \times 3$ để ánh xạ phi tuyến từ tọa độ phẳng Camera $(x, y)$ sang tọa độ phẳng Màn hình hiển thị $(X_{\text{screen}}, Y_{\text{screen}})$ độ phân giải $1920 \times 1080$.
* **Tối ưu hóa toán học:** Hệ phương trình tuyến tính 8 biến thiết lập từ 4 điểm Calibration góc được giải trực tiếp bằng **phương pháp khử Gauss-Jordan**. Toàn bộ quá trình tính toán được nâng cấp lên kiểu dữ liệu số thực độ chính xác kép (`double`) thay vì `float` thông thường, giúp triệt tiêu hoàn toàn sai số tích lũy của phép chia máy tính, loại bỏ hiện tượng giật/trôi con trỏ chuột ở khu vực rìa màn hình.

#### B. Giải thuật Ép góc Chuột (Mouse Absolute Synchronization Hack)
* Giao thức chuột Bluetooth thông thường truyền nhận dữ liệu theo gia số dịch chuyển tương đối ($\Delta x, \Delta y$). Điều này gây hiện tượng trôi tọa độ tích lũy theo thời gian so với vị trí điểm laser tuyệt đối.
* **Giải pháp khắc phục:** Khi khởi tạo hệ thống (`!mouse_initialized`), S3 sẽ phát liên tục 20 lệnh dịch chuyển kịch biên `bleMouse.move(-127, -127)` nhằm đưa con trỏ hệ điều hành Windows về góc tọa độ tuyệt đối $(0,0)$. Sau đó, hệ thống tính toán khoảng cách vector từ $(0,0)$ đến điểm định vị thực tế và dịch chuyển bước tịnh tiến chuột tới mục tiêu để thiết lập mốc đồng bộ hóa ban đầu mượt mà.

#### C. Bộ lọc thông thấp bậc nhất (Low-pass Filter / Exponential Moving Average)
* Nhằm triệt tiêu tần số nhiễu do hiện tượng rung tay sinh lý của người dùng, tọa độ đầu ra được lọc qua bộ lọc thông thấp số:

$$
X_{\text{smooth}} = \alpha \cdot X_{\text{new}} + (1 - \alpha) \cdot X_{\text{old}}
$$

* Giá trị $\alpha = 0.7\text{f}$ được lựa chọn sau khi thực hiện đánh giá đồ thị hóa sai số, giúp cân bằng tối ưu giữa độ mượt di chuyển và độ trễ đáp ứng của con trỏ chuột.

---

### 3. Module `ESP32_Pen_TX`: Quản lý Năng lượng Tiết kiệm nguồn sâu

Module nằm bên trong thân bút cầm tay, xử lý đọc dữ liệu ngoại vi (Nút bấm cơ học, Encoder vòng cuộn) và quản lý tối ưu hóa năng lượng để kéo dài tuổi thọ sử dụng pin.

#### A. Thuật toán Quản lý Trạng thái Nguồn (Power State Machine)
Hệ thống giám sát thời gian rỗi (`idle = millis() - s_last_activity`) để tự động chuyển đổi cấu hình phần cứng theo 3 tầng lớp:
1. **Active Mode:** Hoạt động toàn công suất, kiểm tra quét nút bấm liên tục mỗi `5ms`.
2. **Light Sleep Mode ($idle > 5 \text{ phút}$):** Phát gói tin thông báo trạng thái ngủ (`state = 1`) sang S3 để cập nhật giao diện OLED. Thực hiện tắt khối RF phát sóng (`esp_wifi_stop()`) và đưa CPU vào chế độ ngủ nhẹ. Hệ thống duy trì nguồn cho bộ nhớ RAM để giữ trạng thái chương trình và cấu hình đánh thức lập tức qua ngắt mức thấp của các chân GPIO (`gpio_wakeup_enable`).
3. **Deep Sleep Mode ($idle > 30 \text{ phút}$):** Tiến hành ngắt toàn bộ cấu hình wakeup của các chân ngoại vi thông thường, chỉ duy trì duy nhất ngắt phần cứng `ext0` trên chân nút nhấn hiệu chuẩn (`BTN_CAL`). Vi xử lý ngắt nguồn hầu hết các khối nội vi để dòng tiêu thụ hạ xuống mức micro-Ampere ($\mu\text{A}$). Khi người dùng bấm nút Calib, chip sẽ khởi động lại toàn bộ (Hard Reset) để tái hoạt động.

---

## 📂 Sơ đồ cấu trúc Kho lưu trữ Code

```text
Laser-Pen-ESP32/
├── ESP32CAM_Vision/
│   └── main.cpp          <-- Thuật toán Bounded ROI, Sub-pixel Centroid, Quét động ROI
├── ESP32S3_Central/
│   └── main.cpp          <-- Khử Gauss-Jordan tính Homography (double), Lọc Alpha, Bluetooth HID
├── ESP32_Pen_TX/
│   └── main.cpp          <-- Lập lịch Sleep nguồn, Đọc bộ lọc Debounce nút, Đọc Encoder cơ học
└── README.md             <-- Tài liệu mô tả thuật toán hệ thống toàn cục
```

---
```
 Gru
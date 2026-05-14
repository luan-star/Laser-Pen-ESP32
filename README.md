# Mã nguồn Đồ án: Thiết bị Bút Thông Minh

**Sinh viên thực hiện:** Nguyễn Hữu Luân (20200253)
**Giáo viên hướng dẫn:** ThS. Đỗ Quốc Minh Đăng

Đây là kho lưu trữ mã nguồn cho hệ thống Bút thông minh (Smart Pen), bao gồm 3 module hoạt động đồng bộ với nhau.

## 📁 Cấu trúc Module

### 1. `ESP32CAM_Vision` (Xử lý ảnh & Tracking)
- Đảm nhiệm việc thu thập khung hình từ camera OV2640.
- **Tính năng nổi bật (Cập nhật mới):** Thuật toán **Bounded ROI Tracking**. 
  - Khi bám dấu thành công: Quét vùng kích thước động dựa trên vector vận tốc (Kinematic prediction).
  - Khi mất dấu (Lost tracking): Thay vì quét toàn bộ $320\times240$, hệ thống chỉ quét trong **Vùng Calibration + 10 pixel dung sai**, giúp giảm thiểu tối đa tài nguyên tính toán và giữ FPS ổn định.
- Gửi tọa độ $(x,y)$ về S3 qua UART.

### 2. `ESP32S3_Central` (Trạm xử lý trung tâm)
- Nhận tọa độ từ Camera qua UART và dữ liệu nút bấm từ Bút qua ESP-NOW.
- Tính toán ma trận **Homography** (sử dụng phương pháp khử Gauss-Jordan với kiểu dữ liệu `double` để chống sai số/trôi chuột).
- Áp dụng bộ lọc Low-pass (Alpha) để nội suy tọa độ mượt mà.
- Đóng gói thành dữ liệu HID BLE gửi lên máy tính.

### 3. `ESP32_Pen_TX` (Bút phát tín hiệu)
- Đọc trạng thái thao tác người dùng (Encoder cuộn, nút bấm click, nút Calib).
- Tích hợp hệ thống quản lý năng lượng thông minh:
  - **Light Sleep:** Tự động tắt WiFi/ESP-NOW sau 5 phút không hoạt động (được đánh thức bằng GPIO).
  - **Deep Sleep:** Ngủ sâu sau 30 phút, bảo toàn tối đa dung lượng pin.

## 🔍 Hướng dẫn xem Code cho Giáo viên
Dạ thưa Thầy, để đánh giá phần thuật toán tối ưu vùng quét mới nhất, Thầy có thể xem trực tiếp tại:
1. Hàm `trackLaser()` trong thư mục `ESP32CAM_Vision` -> Dòng xử lý Logic giới hạn Bounding Box (Cal + 10px).
2. Hàm `computeHomography()` trong thư mục `ESP32S3_Central` -> Logic biến đổi tọa độ và ép góc (0,0) khi khởi tạo chuột.
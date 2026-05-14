/**
 * @file camera_tracking.ino
 * @brief Hệ thống Camera Tracking Laser tốc độ cao cho dự án Bút Thông Minh.
 * 
 * THUẬT TOÁN:
 * 1. Peak Detection: Tìm điểm sáng mạnh nhất trong vùng ROI (Bounding Box).
 * 2. Sub-pixel Centroid (Trọng tâm cường độ): Tính toán tọa độ chính xác bằng 
 *    phương pháp trung bình cộng có trọng số (weighted average) để đạt độ mịn cao.
 * 3. Dynamic ROI: Tự động điều chỉnh vùng quét dựa trên vận tốc hiện tại (Kinematics).
 */


#include <Arduino.h>
#include <math.h> //dùng hàm căn bậc 2 (sqrt) tính vận tốc
#include "esp_camera.h"
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  

#define FRAME_W 320
#define FRAME_H 240
#define PIXEL_STEP 2        // Bước nhảy pixel: Càng lớn xử lý càng nhanh nhưng giảm độ chính xác
#define MIN_PIXELS 4        // Số pixel tối thiểu phải có trên 1 vùng sáng để tính trọng tâm, tránh nhiễu
#define SEARCH_RADIUS 15    // Bán kính tìm kiếm quanh điểm sáng nhất để tính trọng tâm sub-pixel
#define SEND_INTERVAL 30    // Khoảng thời gian tối thiểu giữa 2 lần gửi dữ liệu (ms). Giảm để tăng tần số gửi khi rê nhanh

// ================= CẤU HÌNH NHẬN TỪ S3 =================
int bright_threshold = 200;  
int current_exposure = 50;
int v_factor = 10; // Hệ số vận tốc (VEL). Nhận từ S3 (10 tương đương 1.0)

// ================= BIẾN DỰ ĐOÁN QUỸ ĐẠO =================
unsigned long last_send = 0;
unsigned long last_laser_time = 0; 

float last_x = 0, last_y = 0;
float prev_x = 0, prev_y = 0; 
bool has_tracking = false;
int missed_frames = 0;

// ================= BIẾN LƯU VÙNG QUÉT (BOUNDING BOX) =================
int bound_min_x = 0, bound_max_x = FRAME_W;
int bound_min_y = 0, bound_max_y = FRAME_H;
bool use_bounds = false; // Bật cờ này khi đã Calib xong

camera_config_t config;

#define PACKET_HEADER 0x55AA
typedef struct __attribute__((packed)) { 
    uint16_t header; float x; float y; 
} cam_uart_packet_t;

void setup_camera() {
    config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;   config.pin_d1 = 18;  config.pin_d2 = 19;  config.pin_d3 = 21;
    config.pin_d4 = 36;  config.pin_d5 = 39;  config.pin_d6 = 34;  config.pin_d7 = 35;
    config.pin_xclk = 0; config.pin_pclk = 22; config.pin_vsync = 25; config.pin_href = 23;
    config.pin_sccb_sda = 26; config.pin_sccb_scl = 27;
    config.pin_pwdn = 32; config.pin_reset = -1;
    
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 2; 
    config.grab_mode = CAMERA_GRAB_LATEST; 
    
    esp_camera_init(&config);
    
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_brightness(s, -2); s->set_contrast(s, 2); s->set_saturation(s, -2); 
        s->set_special_effect(s, 2); s->set_whitebal(s, 0); s->set_awb_gain(s, 0); 
        s->set_exposure_ctrl(s, 0); s->set_aec2(s, 0); s->set_ae_level(s, -2); 
        s->set_aec_value(s, current_exposure);
        s->set_gain_ctrl(s, 0); s->set_agc_gain(s, 0);
    }
}

// ========================================================
// HÀM NHẬN CẤU HÌNH TỪ ESP32-S3
// ========================================================
void readUartConfig() {
    while (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        if(data.length() < 1) continue; 
        
        char cmd = data.charAt(0);
        
        if (cmd == 'C') {
            // Nhấn nút chuyển về trạng thái CAL -> Hủy giới hạn, quét toàn camera
            use_bounds = false; 
        }
        else if (cmd == 'B') {
            // Nhận vùng Calib sau khi tính xong (VD: B10,300,20,200)
            int comma1 = data.indexOf(',', 1);
            int comma2 = data.indexOf(',', comma1 + 1);
            int comma3 = data.indexOf(',', comma2 + 1);
            
            if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
                int min_x = data.substring(1, comma1).toInt();
                int max_x = data.substring(comma1 + 1, comma2).toInt();
                int min_y = data.substring(comma2 + 1, comma3).toInt();
                int max_y = data.substring(comma3 + 1).toInt();
                
                // MỞ RỘNG THÊM 10 PIXEL XUNG QUANH VÀ KHÔNG CHO VƯỢT QUÁ KHUNG ẢNH
                bound_min_x = max(0, min_x - 10);
                bound_max_x = min(FRAME_W - 1, max_x + 10);
                bound_min_y = max(0, min_y - 10);
                bound_max_y = min(FRAME_H - 1, max_y + 10);
                
                use_bounds = true;
            }
        }
        else if (cmd == 'E') {
            current_exposure = data.substring(1).toInt();
            sensor_t *s = esp_camera_sensor_get();
            if (s) s->set_aec_value(s, current_exposure);
        } 
        else if (cmd == 'T') { bright_threshold = data.substring(1).toInt(); } 
        else if (cmd == 'V') { v_factor = data.substring(1).toInt(); } // Vẫn nhận biến Vận tốc
    }
}

// ========================================================
// THUẬT TOÁN KINEMATIC BÊN TRONG BOUNDING BOX
// ========================================================
bool trackLaser(uint8_t *img, float &out_x, float &out_y) {
    int sx, ex, sy, ey;
    
    // TẠO GIỚI HẠN TỐI ĐA (Nếu chưa Calib thì là Full màn, nếu đã Calib thì là Khung + 10px)
    int max_sx = use_bounds ? bound_min_x : 0;
    int max_ex = use_bounds ? bound_max_x : FRAME_W;
    int max_sy = use_bounds ? bound_min_y : 0;
    int max_ey = use_bounds ? bound_max_y : FRAME_H;

    // BƯỚC 1: XÁC ĐỊNH VÙNG QUÉT
    if(has_tracking) {
        // --- 1.1 ĐANG CÓ TRACKING: Chạy thuật toán ROI vận tốc ---
        float v_x = last_x - prev_x; 
        float v_y = last_y - prev_y;
        float velocity = sqrt(v_x*v_x + v_y*v_y);
        
        int dynamic_roi = 15 + (int)(velocity * ((float)v_factor / 10.0));
        dynamic_roi = constrain(dynamic_roi, 15, 100); 

        // Tăng damping lên 0.8 để tránh ROI bị tụt lại phía sau quá nhiều khi rê nhanh
        float damping = 0.8f; 
        int predict_x = (int)(last_x + (v_x * damping));
        int predict_y = (int)(last_y + (v_y * damping));

        // Lồng ghép (Clamp) Dynamic ROI vào bên trong Vùng Calib
        // Đảm bảo dù dự đoán văng ra ngoài, ô quét vẫn bị giới hạn lại bởi max_sx, max_ex...
        sx = max(max_sx, predict_x - dynamic_roi); 
        ex = min(max_ex - 1, predict_x + dynamic_roi);
        sy = max(max_sy, predict_y - dynamic_roi); 
        ey = min(max_ey - 1, predict_y + dynamic_roi);
        
    } else { 
        // --- 1.2 MẤT TRACKING: Không quét toàn khung ảnh nữa! ---
        // Chỉ quét trong Vùng Calib + 10px (Nhanh hơn rất nhiều so với quét 320x240)
        sx = max_sx; 
        ex = max_ex; 
        sy = max_sy; 
        ey = max_ey; 
    }

    // BƯỚC 2: TÌM ĐIỂM SÁNG NHẤT (PEAK DETECTION) TRONG VÙNG ĐÃ CHỌN
    int peak_x = -1, peak_y = -1;
    uint8_t max_val = bright_threshold;

    for(int y = sy; y < ey; y += PIXEL_STEP) {
        int row_offset = y * FRAME_W;
        for(int x = sx; x < ex; x += PIXEL_STEP) {
            uint8_t val = img[row_offset + x];
            if(val > max_val) { max_val = val; peak_x = x; peak_y = y; }
        }
    }
    if(peak_x < 0) return false;

    // BƯỚC 3: TÍNH TRỌNG TÂM (SUB-PIXEL) 
    // Vẫn phải kẹp giới hạn không cho văng ra khỏi biên (max_sx, max_ex...)
    int start_x = max(max_sx, peak_x - SEARCH_RADIUS); 
    int end_x = min(max_ex - 1, peak_x + SEARCH_RADIUS);
    int start_y = max(max_sy, peak_y - SEARCH_RADIUS); 
    int end_y = min(max_ey - 1, peak_y + SEARCH_RADIUS);
    
    uint32_t sum_x_I = 0, sum_y_I = 0, sum_I = 0;
    
    for(int y = start_y; y <= end_y; y++) {
        int row_offset = y * FRAME_W;
        for(int x = start_x; x <= end_x; x++) {
            uint8_t val = img[row_offset + x];
            if(val > bright_threshold) { 
                sum_x_I += x * val; 
                sum_y_I += y * val; 
                sum_I += val; 
            }
        }
    }
    
    if(sum_I < (MIN_PIXELS * bright_threshold / 2)) return false;
    
    out_x = (float)sum_x_I / sum_I; 
    out_y = (float)sum_y_I / sum_I;
    return true;
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    setCpuFrequencyMhz(240); 
    Serial.begin(115200); 
    setup_camera();
}

void loop() {
    readUartConfig(); 

    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb) { vTaskDelay(pdMS_TO_TICKS(5)); return; }

    float cx, cy;
    bool detected = trackLaser(fb->buf, cx, cy);
    unsigned long now = millis();
    
    if(detected) {
        missed_frames = 0; 
        
        // CẬP NHẬT LỊCH SỬ TỌA ĐỘ CHO LẦN DỰ ĐOÁN TIẾP THEO
        prev_x = last_x; prev_y = last_y; 
        last_x = cx;     last_y = cy;     
        
        has_tracking = true; last_laser_time = now; 
        
        if(now - last_send > SEND_INTERVAL) {
            cam_uart_packet_t pkt = {PACKET_HEADER, cx, cy};
            Serial.write((uint8_t *)&pkt, sizeof(pkt)); last_send = now;
        }
    } else {
        if(++missed_frames > 2) has_tracking = false;
        
        if(now - last_send > 1000) {
            cam_uart_packet_t pkt = {PACKET_HEADER, -1, -1};
            Serial.write((uint8_t *)&pkt, sizeof(pkt)); last_send = now;
        }
    }
    
    esp_camera_fb_return(fb);

    if (now - last_laser_time > 3000) { vTaskDelay(pdMS_TO_TICKS(300)); } 
    else { vTaskDelay(pdMS_TO_TICKS(1)); } 
}
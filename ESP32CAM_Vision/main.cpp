/**
 * @file main.cpp
 * @brief High-speed Laser Tracking Vision System for the Laser Pen project.
 * * @author Nguyen Huu Luan (ID: 20200253)
 * @institution VNUHCM - University of Science, Dept. of Electronics & Telecommunications
 * * CORE ALGORITHMS:
 * 1. Bounded ROI Recovery: Confines the scan area to a predefined workspace boundary 
 * when tracking is lost, ensuring rapid recovery and consistent maximum FPS.
 * 2. Peak Detection & Sub-pixel Centroid: Identifies the brightest local pixel and 
 * calculates the intensity-weighted center of mass for high-precision, sub-pixel accuracy.
 * 3. Dynamic Kinematic ROI: Automatically adjusts the tracking window radius in real-time 
 * based on the velocity vector of the moving laser point.
 * 4. Remote Laser Sync: Listens to ESP-NOW commands to safely toggle the laser diode 
 * during calibration and tracking states.
 */


#include <Arduino.h>
#include <math.h>
#include "esp_camera.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define FRAME_W 320
#define FRAME_H 240
#define PIXEL_STEP 2
#define MIN_PIXELS 4
#define SEARCH_RADIUS 15
#define SEND_INTERVAL 30

// ================= LASER CROSS PIN (on CAM) =================
#define LASER_CROSS_PIN 13

// MAC address of ESP32-S3 Receiver
static uint8_t s_s3_mac[] = {0x44, 0x1B, 0xF6, 0x8C, 0x1A, 0x1C};

// ================= CONFIGURATION RECEIVED FROM S3 =================
int bright_threshold = 200;
int current_exposure = 50;
int v_factor = 10;

// ================= TRAJECTORY PREDICTION VARIABLES =================
unsigned long last_send = 0;
unsigned long last_laser_time = 0;

float last_x = 0, last_y = 0;
float prev_x = 0, prev_y = 0;
bool has_tracking = false;
int missed_frames = 0;

// ================= SCAN AREA BOUNDARY VARIABLES =================
int bound_min_x = 0, bound_max_x = FRAME_W;
int bound_min_y = 0, bound_max_y = FRAME_H;
bool use_bounds = false;

// Current laser cross state
bool laser_on = false;

camera_config_t config;

// ================= SYNCHRONIZED SEND/RECEIVE STRUCTURES =================
typedef struct __attribute__((packed))
{
    float x;
    float y;
} cam_packet_t;

typedef struct __attribute__((packed))
{
    char cmd;
    int val_ex;
    int val_th;
    int val_v;
    int val_rf;
} cam_config_packet_t;

// ================= LASER CROSS CONTROL =================
void setLaser(bool on)
{
    laser_on = on;
    digitalWrite(LASER_CROSS_PIN, on ? HIGH : LOW);
}

// ================= PROCESS CONFIG PACKET FROM S3 =================
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    if (len == sizeof(cam_config_packet_t))
    {
        cam_config_packet_t cfg;
        memcpy(&cfg, incomingData, sizeof(cfg));

        char cmd = cfg.cmd;

        if (cmd == 'C')
        {
            // Reset to full frame scan, turn on laser to get calibration points
            use_bounds = false;
            setLaser(true);
        }
        else if (cmd == 'B')
        {
            int min_x = cfg.val_ex;
            int max_x = cfg.val_th;
            int min_y = cfg.val_v;
            int max_y = cfg.val_rf;

            bound_min_x = max(0, min_x - 10);
            bound_max_x = min(FRAME_W - 1, max_x + 10);
            bound_min_y = max(0, min_y - 10);
            bound_max_y = min(FRAME_H - 1, max_y + 10);

            use_bounds = true;
        }
        else if (cmd == 'E')
        {
            current_exposure = cfg.val_ex;
            sensor_t *s = esp_camera_sensor_get();
            if (s)
                s->set_aec_value(s, current_exposure);
        }
        else if (cmd == 'T')
        {
            bright_threshold = cfg.val_ex;
        }
        else if (cmd == 'V')
        {
            v_factor = cfg.val_ex;
        }
        else if (cmd == 'L')
        {
            setLaser(cfg.val_ex != 0);
        }
    }
}

void setup_camera()
{
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;
    config.pin_d1 = 18;
    config.pin_d2 = 19;
    config.pin_d3 = 21;
    config.pin_d4 = 36;
    config.pin_d5 = 39;
    config.pin_d6 = 34;
    config.pin_d7 = 35;
    config.pin_xclk = 0;
    config.pin_pclk = 22;
    config.pin_vsync = 25;
    config.pin_href = 23;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;
    config.pin_pwdn = 32;
    config.pin_reset = -1;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;  

    esp_camera_init(&config);

    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr)
    {
        s->set_brightness(s, -2);
        s->set_contrast(s, 2);
        s->set_saturation(s, -2);
        s->set_special_effect(s, 2);
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_aec2(s, 0);
        s->set_ae_level(s, -2);
        s->set_aec_value(s, current_exposure);
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, 0);
    }
}

// ================= LASER TRACKING ALGORITHM =================
bool trackLaser(uint8_t *img, float &out_x, float &out_y)
{
    int sx, ex, sy, ey;
    int max_sx = use_bounds ? bound_min_x : 0;
    int max_ex = use_bounds ? bound_max_x : FRAME_W;
    int max_sy = use_bounds ? bound_min_y : 0;
    int max_ey = use_bounds ? bound_max_y : FRAME_H;

    if (has_tracking)
    {
        float v_x = last_x - prev_x;
        float v_y = last_y - prev_y;
        float velocity = sqrt(v_x * v_x + v_y * v_y);
        int dynamic_roi = 15 + (int)(velocity * ((float)v_factor / 10.0));
        dynamic_roi = constrain(dynamic_roi, 15, 100);

        float damping = 0.8f;
        int predict_x = (int)(last_x + (v_x * damping));
        int predict_y = (int)(last_y + (v_y * damping));

        sx = max(max_sx, predict_x - dynamic_roi);
        ex = min(max_ex - 1, predict_x + dynamic_roi);
        sy = max(max_sy, predict_y - dynamic_roi);
        ey = min(max_ey - 1, predict_y + dynamic_roi);
    }
    else
    {
        sx = max_sx;
        ex = max_ex;
        sy = max_sy;
        ey = max_ey;
    }

    int peak_x = -1, peak_y = -1;
    uint8_t max_val = bright_threshold;

    for (int y = sy; y < ey; y += PIXEL_STEP)
    {
        int row_offset = y * FRAME_W;
        for (int x = sx; x < ex; x += PIXEL_STEP)
        {
            uint8_t val = img[row_offset + x];
            if (val > max_val)
            {
                max_val = val;
                peak_x = x;
                peak_y = y;
            }
        }
    }
    if (peak_x < 0)
        return false;

    int start_x = max(max_sx, peak_x - SEARCH_RADIUS);
    int end_x = min(max_ex - 1, peak_x + SEARCH_RADIUS);
    int start_y = max(max_sy, peak_y - SEARCH_RADIUS);
    int end_y = min(max_ey - 1, peak_y + SEARCH_RADIUS);

    uint32_t sum_x_I = 0, sum_y_I = 0, sum_I = 0;

    for (int y = start_y; y <= end_y; y++)
    {
        int row_offset = y * FRAME_W;
        for (int x = start_x; x <= end_x; x++)
        {
            uint8_t val = img[row_offset + x];
            if (val > bright_threshold)
            {
                sum_x_I += x * val;
                sum_y_I += y * val;
                sum_I += val;
            }
        }
    }

    if (sum_I < (MIN_PIXELS * bright_threshold / 2))
        return false;

    out_x = (float)sum_x_I / sum_I;
    out_y = (float)sum_y_I / sum_I;
    return true;
}

// ================= SETUP =================
void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    setCpuFrequencyMhz(240);
    Serial.begin(115200);

    // Initialize laser cross pin — turn off by default on boot
    pinMode(LASER_CROSS_PIN, OUTPUT);
    setLaser(false);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, s_s3_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    setup_camera();
}

// ================= LOOP =================
void loop()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        return;
    }

    float cx, cy;
    bool detected = trackLaser(fb->buf, cx, cy);
    unsigned long now = millis();

    if (detected)
    {
        missed_frames = 0;
        prev_x = last_x;
        prev_y = last_y;
        last_x = cx;
        last_y = cy;
        has_tracking = true;
        last_laser_time = now;

        if (now - last_send > SEND_INTERVAL)
        {
            cam_packet_t pkt = {cx, cy};
            esp_now_send(s_s3_mac, (uint8_t *)&pkt, sizeof(pkt));
            last_send = now;
        }
    }
    else
    {
        if (++missed_frames > 2)
            has_tracking = false;

        if (now - last_send > 1000)
        {
            cam_packet_t pkt = {-1.0f, -1.0f};
            esp_now_send(s_s3_mac, (uint8_t *)&pkt, sizeof(pkt));
            last_send = now;
        }
    }

    esp_camera_fb_return(fb);

    if (now - last_laser_time > 3000)
    {
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
/**
 * @file main.cpp
 * @brief Central Processing Hub & HID Controller for the Laser Pen project.
 * * @author Nguyen Huu Luan (ID: 20200253)
 * @institution VNUHCM - University of Science, Dept. of Electronics & Telecommunications
 * * CORE ALGORITHMS & FEATURES:
 * 1. Planar Homography Transformation: Solves an 8-variable linear equation system using 
 * Gauss-Jordan elimination (double precision) to non-linearly map Camera coordinates 
 * to Display coordinates (1920x1080).
 * 2. Absolute Mouse Synchronization: Fires a rapid sequence of bounding commands to force 
 * the OS cursor to the absolute origin (0,0) before applying precise BLE HID offsets.
 * 3. Exponential Moving Average (EMA): Applies a digital low-pass filter (Alpha tuning) 
 * to the output coordinates to eliminate physiological hand tremors.
 * 4. ESP-NOW Telemetry & UI: Manages full-duplex communication with the Vision module 
 * and Pen TX, updating the local I2C OLED dashboard with real-time states and FPS.
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Preferences.h>
#include <BleMouse.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= SAFE PIN DECLARATIONS =================
// Button for OLED menu navigation function
#define BTN_MODE_PIN 1
#define BTN_UP_PIN 2
#define BTN_DOWN_PIN 21

// Mouse buttons integrated on pen body
#define BTN_LEFT_PIN 14
#define BTN_RIGHT_PIN 11
#define BTN_NEXT_PIN 16
#define BTN_PREV_PIN 17
#define BTN_CAL_PIN 18

// Page scroll encoder
#define ENC_A_PIN 12
#define ENC_B_PIN 13

// Battery pin reading
#define LOCAL_BAT_PIN 10

BleMouse bleMouse("LASER PEN", "Espressif", 100);

// MAC address of ESP32-CAM
static uint8_t s_cam_mac[] = {0x3C, 0x61, 0x05, 0x23, 0x8F, 0xD8};

// ================= ESP-NOW SYNCHRONIZATION DATA STRUCTURES =================
typedef struct __attribute__((packed))
{
    float x;
    float y;
} cam_packet_t;

typedef struct __attribute__((packed))
{
    char cmd; // 'C', 'B', 'E', 'T', 'V', 'L'
    int val_ex;
    int val_th;
    int val_v;
    int val_rf;
} cam_config_packet_t;

typedef struct __attribute__((packed))
{
    int8_t scroll;
    uint8_t buttons;
    uint8_t battery;
    uint8_t state;
} pen_packet_t;

// ================= GLOBAL VARIABLES & STATE =================
enum State
{
    WAIT_SYNC,
    CAL_PT_1,
    CAL_PT_2,
    CAL_PT_3,
    CAL_PT_4,
    TRACKING
};
volatile State state = WAIT_SYNC;
volatile State next_state = CAL_PT_1;
volatile bool is_rebooting = false;

volatile uint8_t local_battery = 0;
volatile uint32_t last_tune_time = 0;

enum TuneMode
{
    MODE_EXP,
    MODE_THR,
    MODE_ALP,
    MODE_VEL
};
TuneMode current_tune_mode = MODE_EXP;

int val_exposure = 50;
int val_threshold = 200;
float current_alpha = 0.7f;
int current_v_fac = 10;

const int screen_w = 1920;
const int screen_h = 1080;

bool mouse_initialized = false;
float exact_mouse_x = 0, exact_mouse_y = 0;
float remainder_x = 0, remainder_y = 0;
float smooth_x = 0, smooth_y = 0;
volatile float current_cam_x = 0, current_cam_y = 0;

Preferences preferences;
QueueHandle_t penQueue, camQueue, btnQueue;
SemaphoreHandle_t hidMutex;

uint8_t lastButtons = 0;
volatile uint8_t g_display_buttons = 0;
volatile uint32_t frame_count = 0, current_fps = 0, last_fps_time = 0;
volatile uint32_t last_cam_time = 0;

float H[3][3];
float cam_pts[4][2];

#define OLED_SLEEP_TIMEOUT_MS 60000
volatile uint32_t last_system_activity = 0;

// Local encoder handling variables
static volatile int8_t s_enc_delta = 0;
static volatile uint8_t s_enc_last = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encoderISR()
{
    uint8_t curr_state = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
    uint8_t idx = (s_enc_last << 2) | curr_state;
    int8_t move = ENC_TABLE[idx];
    if (move != 0)
    {
        portENTER_CRITICAL_ISR(&s_mux);
        s_enc_delta += move;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
    s_enc_last = curr_state;
}

// ================= RECEIVE DATA VIA ESP-NOW =================
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
    if (len == sizeof(cam_packet_t))
    {
        cam_packet_t packet;
        memcpy(&packet, incomingData, sizeof(packet));
        xQueueOverwriteFromISR(camQueue, &packet, NULL);
        last_cam_time = millis();
    }
}

void displayTask(void *pv);
void mainTask(void *pv);
void localInputTask(void *pv);
void penTask(void *pv);
void tuningTask(void *pv);
void computeHomography();
void updateBootScreen(int progress, const char *message);
void sendTuningToCam(char cmd, int v1, int v2, int v3, int v4);
void sendLaserToCam(bool on);
void drawBatteryIcon(int x, int y, uint8_t percent, uint8_t state, bool is_lost);
void updateLocalBattery();

// ================= SETUP =================
void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(BTN_MODE_PIN, INPUT_PULLUP);
    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

    pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(BTN_PREV_PIN, INPUT_PULLUP);
    pinMode(BTN_CAL_PIN, INPUT_PULLUP);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

    Wire.begin();
    Wire.setClock(400000);
    display.begin(0x3C, true);
    updateBootScreen(10, "Init IO...");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    updateBootScreen(40, "Init ESP-NOW...");
    if (esp_now_init() != ESP_OK)
        return;
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, s_cam_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    updateBootScreen(70, "Init Bluetooth...");
    bleMouse.begin();

    penQueue = xQueueCreate(10, sizeof(pen_packet_t));
    camQueue = xQueueCreate(1, sizeof(cam_packet_t));
    btnQueue = xQueueCreate(5, sizeof(uint8_t));
    hidMutex = xSemaphoreCreateMutex();

    preferences.begin("laserpen", false);
    val_exposure = preferences.getInt("exp", 50);
    val_threshold = preferences.getInt("thr", 200);
    current_alpha = preferences.getFloat("alp", 0.7f);
    current_v_fac = preferences.getInt("vfac", 10);

    int bbox[4];
    if (preferences.getBytesLength("matrix_H") == sizeof(H) && preferences.getBytesLength("bbox") == sizeof(bbox))
    {
        preferences.getBytes("matrix_H", &H, sizeof(H));
        next_state = TRACKING;
    }
    else
    {
        next_state = CAL_PT_1;
    }

    updateLocalBattery();
    updateBootScreen(100, "System Ready!");
    delay(500);

    xTaskCreatePinnedToCore(displayTask, "DSPL", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(tuningTask, "TUNE", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(localInputTask, "INPUT", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(penTask, "PEN", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(mainTask, "MAIN", 8192, NULL, 2, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }

// ================= READ BATTERY PERCENTAGE =================
void updateLocalBattery()
{
    analogSetPinAttenuation(LOCAL_BAT_PIN, ADC_11db);
    int mv = 0;
    for (int i = 0; i < 10; i++)
    {
        mv += analogReadMilliVolts(LOCAL_BAT_PIN);
        delay(2);
    }
    mv /= 10;
    int max_mv = 4200;
    int min_mv = 3300;
    int percent = map(mv, min_mv, max_mv, 0, 100);
    local_battery = constrain(percent, 0, 100);
}

// ================= BUTTON DEBOUNCE =================
#define NUM_BUTTONS 5
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {BTN_LEFT_PIN, BTN_RIGHT_PIN, BTN_NEXT_PIN, BTN_PREV_PIN, BTN_CAL_PIN};
const uint8_t BUTTON_MASKS[NUM_BUTTONS] = {0x01, 0x02, 0x04, 0x08, 0x10};

void localInputTask(void *pv)
{
    uint8_t last_raw_state = 0;
    uint8_t debounced_state = 0;
    uint8_t last_sent_buttons = 0;
    uint32_t last_debounce_time[NUM_BUTTONS] = {0};
    const uint32_t debounce_delay_ms = 25;

    for (int i = 0; i < NUM_BUTTONS; i++)
    {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    while (1)
    {
        pen_packet_t local_pkt = {0};
        uint32_t now = millis();

        portENTER_CRITICAL(&s_mux);
        local_pkt.scroll = s_enc_delta;
        s_enc_delta = 0;
        portEXIT_CRITICAL(&s_mux);

        local_pkt.scroll = constrain(local_pkt.scroll, -5, 5);

        for (int i = 0; i < NUM_BUTTONS; i++)
        {
            bool current_reading = (digitalRead(BUTTON_PINS[i]) == LOW);
            bool last_raw_bit = (last_raw_state & BUTTON_MASKS[i]) ? true : false;

            if (current_reading != last_raw_bit)
            {
                last_debounce_time[i] = now;
                if (current_reading)
                    last_raw_state |= BUTTON_MASKS[i];
                else
                    last_raw_state &= ~BUTTON_MASKS[i];
            }

            if ((now - last_debounce_time[i]) >= debounce_delay_ms)
            {
                bool debounced_bit = (debounced_state & BUTTON_MASKS[i]) ? true : false;
                if (current_reading != debounced_bit)
                {
                    if (current_reading)
                        debounced_state |= BUTTON_MASKS[i];
                    else
                        debounced_state &= ~BUTTON_MASKS[i];
                }
            }
        }

        local_pkt.buttons = debounced_state;
        g_display_buttons = debounced_state; // update for displayTask
        bool should_send = (local_pkt.buttons != last_sent_buttons) || (local_pkt.scroll != 0);

        if (should_send)
        {
            local_pkt.battery = local_battery;
            local_pkt.state = 0;
            xQueueSend(penQueue, &local_pkt, portMAX_DELAY);
            last_sent_buttons = local_pkt.buttons;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================= TUNING TASK =================
void tuningTask(void *pv)
{
    uint32_t last_mode_time = 0, last_up_time = 0, last_down_time = 0, next_save_time = 0;
    vTaskDelay(pdMS_TO_TICKS(2000));

    sendTuningToCam('E', val_exposure, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    sendTuningToCam('T', val_threshold, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    sendTuningToCam('V', current_v_fac, 0, 0, 0);

    while (1)
    {
        uint32_t now = millis();
        bool changed = false;

        if (digitalRead(BTN_MODE_PIN) == LOW)
        {
            uint32_t press_start = millis();
            bool is_long_press = false;
            while (digitalRead(BTN_MODE_PIN) == LOW)
            {
                if (millis() - press_start > 3000)
                {
                    is_long_press = true;
                    is_rebooting = true;
                    display.clearDisplay();
                    display.drawRoundRect(0, 0, 128, 64, 8, SH110X_WHITE);
                    display.setTextSize(2);
                    display.setCursor(15, 25);
                    display.print("REBOOTING");
                    display.display();
                    delay(1000);
                    ESP.restart();
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (!is_long_press && now - last_mode_time > 300)
            {
                current_tune_mode = (TuneMode)((current_tune_mode + 1) % 4);
                last_mode_time = millis();
                last_tune_time = now;
                last_system_activity = now;
            }
        }

        if (digitalRead(BTN_UP_PIN) == LOW && now - last_up_time > 150)
        {
            if (current_tune_mode == MODE_EXP)
            {
                val_exposure = min(255, val_exposure + 5);
                sendTuningToCam('E', val_exposure, 0, 0, 0);
            }
            else if (current_tune_mode == MODE_THR)
            {
                val_threshold = min(255, val_threshold + 5);
                sendTuningToCam('T', val_threshold, 0, 0, 0);
            }
            else if (current_tune_mode == MODE_ALP)
            {
                current_alpha = min(1.0f, current_alpha + 0.1f);
            }
            else if (current_tune_mode == MODE_VEL)
            {
                current_v_fac = min(50, current_v_fac + 2);
                sendTuningToCam('V', current_v_fac, 0, 0, 0);
            }
            changed = true;
            last_up_time = now;
            last_tune_time = now;
            last_system_activity = now;
        }

        if (digitalRead(BTN_DOWN_PIN) == LOW && now - last_down_time > 150)
        {
            if (current_tune_mode == MODE_EXP)
            {
                val_exposure = max(0, val_exposure - 5);
                sendTuningToCam('E', val_exposure, 0, 0, 0);
            }
            else if (current_tune_mode == MODE_THR)
            {
                val_threshold = max(0, val_threshold - 5);
                sendTuningToCam('T', val_threshold, 0, 0, 0);
            }
            else if (current_tune_mode == MODE_ALP)
            {
                current_alpha = max(0.1f, current_alpha - 0.1f);
            }
            else if (current_tune_mode == MODE_VEL)
            {
                current_v_fac = max(0, current_v_fac - 2);
                sendTuningToCam('V', current_v_fac, 0, 0, 0);
            }
            changed = true;
            last_down_time = now;
            last_tune_time = now;
            last_system_activity = now;
        }

        if (changed && now > next_save_time)
        {
            preferences.putInt("exp", val_exposure);
            preferences.putInt("thr", val_threshold);
            preferences.putFloat("alp", current_alpha);
            preferences.putInt("vfac", current_v_fac);
            next_save_time = now + 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void sendTuningToCam(char cmd, int v1, int v2, int v3, int v4)
{
    cam_config_packet_t config_pkt = {cmd, v1, v2, v3, v4};
    esp_now_send(s_cam_mac, (uint8_t *)&config_pkt, sizeof(config_pkt));
}

// ================= LASER CONTROL (send command to CAM) =================
void sendLaserToCam(bool on)
{
    cam_config_packet_t pkt = {'L', on ? 1 : 0, 0, 0, 0};
    esp_now_send(s_cam_mac, (uint8_t *)&pkt, sizeof(pkt));
}

// ================= OLED DISPLAY =================
struct UI_State
{
    uint32_t fps;
    bool ble_conn;
    uint8_t local_bat;
    State system_state;
    TuneMode tune_mode;
    bool is_tuning;
    int exp, thr;
    float alp;
    int vfac;
    uint8_t buttons; // current button state for drawing hints
};

void drawBatteryIcon(int x, int y, uint8_t percent, uint8_t state, bool is_lost)
{
    display.drawRect(x, y, 22, 10, SH110X_WHITE);
    display.fillRect(x + 22, y + 2, 2, 6, SH110X_WHITE);
    display.fillRect(x + 1, y + 1, 20, 8, SH110X_BLACK);
    if (is_lost)
    {
        display.setCursor(x + 4, y + 1);
        display.print(" X ");
    }
    else
    {
        int fill_w = map(percent, 0, 100, 0, 20);
        display.fillRect(x + 1, y + 1, fill_w, 8, SH110X_WHITE);
    }
}

void displayTask(void *pv)
{
    UI_State last_ui;
    memset(&last_ui, 0, sizeof(UI_State));

    bool force_clear = true;
    bool is_screen_off = false;
    uint32_t last_bat_check = 0;
    last_system_activity = millis();

    while (1)
    {
        if (is_rebooting)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint32_t now = millis();
        if (now - last_bat_check > 60000)
        {
            updateLocalBattery();
            last_bat_check = now;
        }

        if (now - last_system_activity > OLED_SLEEP_TIMEOUT_MS)
        {
            if (!is_screen_off)
            {
                display.clearDisplay();
                display.display();
                is_screen_off = true;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        else
        {
            if (is_screen_off)
            {
                is_screen_off = false;
                force_clear = true;
            }
        }

        if (now - last_fps_time >= 1000)
        {
            current_fps = frame_count;
            frame_count = 0;
            last_fps_time = now;
        }

        UI_State curr_ui;
        memset(&curr_ui, 0, sizeof(UI_State));
        curr_ui.fps = current_fps;
        curr_ui.ble_conn = bleMouse.isConnected();
        curr_ui.local_bat = local_battery;
        curr_ui.system_state = state;
        curr_ui.tune_mode = current_tune_mode;
        curr_ui.is_tuning = (now - last_tune_time < 3000);
        curr_ui.exp = val_exposure;
        curr_ui.thr = val_threshold;
        curr_ui.alp = current_alpha;
        curr_ui.vfac = current_v_fac;
        curr_ui.buttons = g_display_buttons;

        bool changed = force_clear ||
                       (curr_ui.fps != last_ui.fps) ||
                       (curr_ui.ble_conn != last_ui.ble_conn) ||
                       (curr_ui.local_bat != last_ui.local_bat) ||
                       (curr_ui.system_state != last_ui.system_state) ||
                       (curr_ui.tune_mode != last_ui.tune_mode) ||
                       (curr_ui.is_tuning != last_ui.is_tuning) ||
                       (curr_ui.exp != last_ui.exp) ||
                       (curr_ui.thr != last_ui.thr) ||
                       (curr_ui.alp != last_ui.alp) ||
                       (curr_ui.vfac != last_ui.vfac) ||
                       (curr_ui.buttons != last_ui.buttons);

        if (!changed)
        {
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        if (force_clear || curr_ui.is_tuning != last_ui.is_tuning || curr_ui.system_state != last_ui.system_state)
        {
            display.clearDisplay();
            force_clear = false;
        }

        display.setTextColor(SH110X_WHITE, SH110X_BLACK);
        if (curr_ui.system_state == WAIT_SYNC)
        {
            display.setTextSize(1);
            display.setCursor(16, 0);
            display.print("CHECKING SYNC");
            display.drawRoundRect(20, 15, 88, 24, 3, SH110X_WHITE);
            display.setCursor(26, 23);
            display.printf("CAM LINK: %s", (last_cam_time > 0 && (now - last_cam_time < 2000)) ? "OK" : "NO");
            display.setCursor(30, 48);
            display.printf("BATTERY: %d%%", curr_ui.local_bat);
        }
        else
        {
            display.setTextSize(1);
            if (curr_ui.ble_conn)
            {
                display.fillRect(0, 0, 18, 10, SH110X_WHITE);
                display.setTextColor(SH110X_BLACK, SH110X_WHITE);
                display.setCursor(2, 1);
                display.print("BLE");
            }
            else
            {
                display.fillRect(0, 0, 18, 10, SH110X_BLACK);
                display.setTextColor(SH110X_WHITE, SH110X_BLACK);
                display.setCursor(2, 1);
                display.print("---");
            }
            display.setTextColor(SH110X_WHITE, SH110X_BLACK);
            display.setCursor(25, 1);
            display.printf("FPS:%-2d", curr_ui.fps);
            display.setCursor(65, 1);
            display.print("BAT:");
            drawBatteryIcon(90, 0, curr_ui.local_bat, 0, false);
            display.drawLine(0, 12, 128, 12, SH110X_WHITE);

            display.setTextSize(2);
            display.setCursor(16, curr_ui.is_tuning ? 18 : 32);
            switch (curr_ui.system_state)
            {
            case CAL_PT_1:
                display.print("1: TOP-L  ");
                break;
            case CAL_PT_2:
                display.print("2: TOP-R  ");
                break;
            case CAL_PT_3:
                display.print("3: BOT-R  ");
                break;
            case CAL_PT_4:
                display.print("4: BOT-L  ");
                break;
            case TRACKING:
                display.print("TRACKING  ");
                break;
            default:
                break;
            }

            if (curr_ui.is_tuning)
            {
                display.drawLine(0, 36, 128, 36, SH110X_WHITE);
                display.setTextSize(1);
                auto printOption = [](int x, int y, const char *lbl, float v, bool is_f, bool act)
                {
                    if (act)
                    {
                        display.fillRect(x, y - 1, 62, 10, SH110X_WHITE);
                        display.setTextColor(SH110X_BLACK, SH110X_WHITE);
                    }
                    else
                    {
                        display.fillRect(x, y - 1, 62, 10, SH110X_BLACK);
                        display.setTextColor(SH110X_WHITE, SH110X_BLACK);
                    }
                    display.setCursor(x + 2, y);
                    if (is_f)
                        display.printf("%s: %-3.1f", lbl, v);
                    else
                        display.printf("%s: %-3d", lbl, (int)v);
                };
                printOption(0, 40, "EXP", curr_ui.exp, false, curr_ui.tune_mode == MODE_EXP);
                printOption(66, 40, "THR", curr_ui.thr, false, curr_ui.tune_mode == MODE_THR);
                printOption(0, 52, "ALP", curr_ui.alp, true, curr_ui.tune_mode == MODE_ALP);
                printOption(66, 52, "VEL", curr_ui.vfac, false, curr_ui.tune_mode == MODE_VEL);
            }
        }
        display.display();
        last_ui = curr_ui;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void updateBootScreen(int progress, const char *message)
{
    display.clearDisplay();
    display.drawRoundRect(0, 0, 128, 64, 8, SH110X_WHITE);
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(12, 10);
    display.print("SMART PEN");
    display.setTextSize(1);
    display.setCursor(10, 35);
    display.print(message);
    display.drawRect(10, 48, 108, 10, SH110X_WHITE);
    int fillWidth = (104 * progress) / 100;
    if (fillWidth > 0)
        display.fillRect(12, 50, fillWidth, 6, SH110X_WHITE);
    display.display();
}

// ================= MAIN TASK (Tracking Logic & Homography) =================
void mainTask(void *pv)
{
    cam_packet_t camPkt;
    State last_sent_laser_state = WAIT_SYNC; // track to avoid duplicate sends

    while (1)
    {
        if (state == WAIT_SYNC)
        {
            if (last_cam_time > 0 && millis() - last_cam_time < 2000)
            {
                delay(1000);
                state = next_state;
                if (state == TRACKING)
                {
                    int bbox[4];
                    if (preferences.getBytes("bbox", &bbox, sizeof(bbox)))
                    {
                        sendTuningToCam('B', bbox[0], bbox[1], bbox[2], bbox[3]);
                    }
                    else
                    {
                        sendTuningToCam('C', 0, 0, 0, 0);
                    }
                    // Turn off laser when entering tracking from sync
                    sendLaserToCam(false);
                    last_sent_laser_state = TRACKING;
                }
                else
                {
                    sendTuningToCam('C', 0, 0, 0, 0);
                    // Turn on laser when starting calibration
                    sendLaserToCam(true);
                    last_sent_laser_state = CAL_PT_1;
                }
            }
            vTaskDelay(100);
            continue;
        }

        // Send laser state when state changes
        bool in_cal = (state >= CAL_PT_1 && state <= CAL_PT_4);
        bool last_in_cal = (last_sent_laser_state >= CAL_PT_1 && last_sent_laser_state <= CAL_PT_4);
        if (in_cal != last_in_cal)
        {
            sendLaserToCam(in_cal);
            last_sent_laser_state = state;
        }

        if (xQueueReceive(camQueue, &camPkt, 5))
        {
            frame_count++;
            current_cam_x = camPkt.x;
            current_cam_y = camPkt.y;

            if (state == TRACKING && current_cam_x != -1 && current_cam_y != -1)
            {
                float denom = H[2][0] * current_cam_x + H[2][1] * current_cam_y + H[2][2];
                denom = (fabsf(denom) < 0.0001f) ? 0.0001f : denom;
                float inv_denom = 1.0f / denom;
                float sx = (H[0][0] * current_cam_x + H[0][1] * current_cam_y + H[0][2]) * inv_denom;
                float sy = (H[1][0] * current_cam_x + H[1][1] * current_cam_y + H[1][2]) * inv_denom;

                if (!mouse_initialized)
                {
                    if (bleMouse.isConnected())
                    {
                        if (xSemaphoreTake(hidMutex, pdMS_TO_TICKS(10)))
                        {
                            for (int i = 0; i < 20; i++)
                                bleMouse.move(-127, -127);
                            xSemaphoreGive(hidMutex);
                        }
                        vTaskDelay(pdMS_TO_TICKS(15));
                        int dx = (int)sx, dy = (int)sy;
                        if (xSemaphoreTake(hidMutex, pdMS_TO_TICKS(10)))
                        {
                            while (dx > 0 || dy > 0)
                            {
                                int step_x = constrain(dx, 0, 127);
                                int step_y = constrain(dy, 0, 127);
                                bleMouse.move(step_x, step_y);
                                dx -= step_x;
                                dy -= step_y;
                            }
                            xSemaphoreGive(hidMutex);
                        }
                    }
                    exact_mouse_x = sx;
                    exact_mouse_y = sy;
                    smooth_x = sx;
                    smooth_y = sy;
                    remainder_x = 0;
                    remainder_y = 0;
                    mouse_initialized = true;
                }
                else
                {
                    exact_mouse_x = current_alpha * sx + (1.0f - current_alpha) * exact_mouse_x;
                    exact_mouse_y = current_alpha * sy + (1.0f - current_alpha) * exact_mouse_y;

                    float dx_float = (exact_mouse_x - smooth_x) + remainder_x;
                    float dy_float = (exact_mouse_y - smooth_y) + remainder_y;
                    int total_dx = (int)dx_float, total_dy = (int)dy_float;
                    remainder_x = dx_float - total_dx;
                    remainder_y = dy_float - total_dy;
                    smooth_x = exact_mouse_x;
                    smooth_y = exact_mouse_y;

                    if ((total_dx != 0 || total_dy != 0) && bleMouse.isConnected())
                    {
                        last_system_activity = millis();
                        if (xSemaphoreTake(hidMutex, pdMS_TO_TICKS(10)))
                        {
                            while (total_dx != 0 || total_dy != 0)
                            {
                                int step_x = constrain(total_dx, -127, 127);
                                int step_y = constrain(total_dy, -127, 127);
                                bleMouse.move(step_x, step_y);
                                total_dx -= step_x;
                                total_dy -= step_y;
                            }
                            xSemaphoreGive(hidMutex);
                        }
                    }
                }
            }
            else if (current_cam_x == -1)
            {
                mouse_initialized = false;
            }
        }

        uint8_t evt;
        if (xQueueReceive(btnQueue, &evt, 0))
        {
            if (state == TRACKING)
            {
                state = CAL_PT_1;
                mouse_initialized = false;
                sendTuningToCam('C', 0, 0, 0, 0);
                // Turn on laser when user presses button to start calibration again
                sendLaserToCam(true);
                last_sent_laser_state = CAL_PT_1;
            }
            else if (state >= CAL_PT_1 && state <= CAL_PT_4)
            {
                if (current_cam_x != -1 && current_cam_y != -1)
                {
                    int pt_idx = state - CAL_PT_1;
                    cam_pts[pt_idx][0] = current_cam_x;
                    cam_pts[pt_idx][1] = current_cam_y;
                    if (state == CAL_PT_4)
                    {
                        computeHomography();
                        float min_x = 9999, max_x = -9999, min_y = 9999, max_y = -9999;
                        for (int i = 0; i < 4; i++)
                        {
                            if (cam_pts[i][0] < min_x)
                                min_x = cam_pts[i][0];
                            if (cam_pts[i][0] > max_x)
                                max_x = cam_pts[i][0];
                            if (cam_pts[i][1] < min_y)
                                min_y = cam_pts[i][1];
                            if (cam_pts[i][1] > max_y)
                                max_y = cam_pts[i][1];
                        }
                        int bbox[4] = {(int)min_x, (int)max_x, (int)min_y, (int)max_y};
                        preferences.putBytes("bbox", &bbox, sizeof(bbox));
                        sendTuningToCam('B', bbox[0], bbox[1], bbox[2], bbox[3]);
                        // Turn off laser when calibration complete, switch to tracking
                        sendLaserToCam(false);
                        last_sent_laser_state = TRACKING;
                        state = TRACKING;
                    }
                    else
                    {
                        state = (State)(state + 1);
                        // Keep laser on throughout CAL process
                        last_sent_laser_state = state;
                    }
                }
            }
        }
    }
}

// ================= MOUSE KEY PROCESSING TASK =================
void penTask(void *pv)
{
    pen_packet_t packet;
    while (1)
    {
        if (xQueueReceive(penQueue, &packet, portMAX_DELAY))
        {
            if (bleMouse.isConnected())
            {
                if (xSemaphoreTake(hidMutex, pdMS_TO_TICKS(10)))
                {
                    if (packet.scroll != 0)
                    {
                        bleMouse.move(0, 0, packet.scroll);
                        last_system_activity = millis();
                    }
                    uint8_t btn_change = packet.buttons ^ lastButtons;
                    if (btn_change)
                    {
                        last_system_activity = millis();
                        if (btn_change & 0x01)
                        {
                            (packet.buttons & 0x01) ? bleMouse.press(MOUSE_LEFT) : bleMouse.release(MOUSE_LEFT);
                        }
                        if (btn_change & 0x02)
                        {
                            (packet.buttons & 0x02) ? bleMouse.press(MOUSE_RIGHT) : bleMouse.release(MOUSE_RIGHT);
                        }
                        if (btn_change & 0x04 && (packet.buttons & 0x04))
                        {
                            bleMouse.move(0, 0, -1);
                        }
                        if (btn_change & 0x08 && (packet.buttons & 0x08))
                        {
                            bleMouse.move(0, 0, 1);
                        }
                        if (btn_change & 0x10 && packet.buttons & 0x10)
                        {
                            uint8_t evt = 1;
                            xQueueSend(btnQueue, &evt, 0);
                        }
                        lastButtons = packet.buttons;
                    }
                    xSemaphoreGive(hidMutex);
                }
            }
        }
    }
}

// ================= CALCULATE HOMOGRAPHY MATRIX =================
void computeHomography()
{
    double dst[4][2] = {{0, 0}, {(double)screen_w, 0}, {(double)screen_w, (double)screen_h}, {0, (double)screen_h}};
    double a[8][9];
    memset(a, 0, sizeof(a));
    for (int i = 0; i < 4; i++)
    {
        double x = cam_pts[i][0], y = cam_pts[i][1];
        double u = dst[i][0], v = dst[i][1];
        a[i * 2][0] = x;
        a[i * 2][1] = y;
        a[i * 2][2] = 1;
        a[i * 2][3] = 0;
        a[i * 2][4] = 0;
        a[i * 2][5] = 0;
        a[i * 2][6] = -x * u;
        a[i * 2][7] = -y * u;
        a[i * 2][8] = u;
        a[i * 2 + 1][0] = 0;
        a[i * 2 + 1][1] = 0;
        a[i * 2 + 1][2] = 0;
        a[i * 2 + 1][3] = x;
        a[i * 2 + 1][4] = y;
        a[i * 2 + 1][5] = 1;
        a[i * 2 + 1][6] = -x * v;
        a[i * 2 + 1][7] = -y * v;
        a[i * 2 + 1][8] = v;
    }
    for (int i = 0; i < 8; i++)
    {
        int pivot = i;
        for (int j = i + 1; j < 8; j++)
            if (fabs(a[j][i]) > fabs(a[pivot][i]))
                pivot = j;
        for (int j = i; j < 9; j++)
        {
            double tmp = a[i][j];
            a[i][j] = a[pivot][j];
            a[pivot][j] = tmp;
        }
        if (fabs(a[i][i]) < 1e-6)
            return;
        for (int j = i + 1; j < 8; j++)
        {
            double factor = a[j][i] / a[i][i];
            for (int k = i; k < 9; k++)
                a[j][k] -= factor * a[i][k];
        }
    }
    double h[8];
    for (int i = 7; i >= 0; i--)
    {
        double sum = 0;
        for (int j = i + 1; j < 8; j++)
            sum += a[i][j] * h[j];
        h[i] = (a[i][8] - sum) / a[i][i];
    }
    H[0][0] = (float)h[0];
    H[0][1] = (float)h[1];
    H[0][2] = (float)h[2];
    H[1][0] = (float)h[3];
    H[1][1] = (float)h[4];
    H[1][2] = (float)h[5];
    H[2][0] = (float)h[6];
    H[2][1] = (float)h[7];
    H[2][2] = 1.0f;

    double norm = sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2] + h[3] * h[3] + h[4] * h[4] + h[5] * h[5] + h[6] * h[6] + h[7] * h[7] + 1.0);
    if (norm > 1e-6)
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                H[i][j] /= (float)norm;

    preferences.putBytes("matrix_H", &H, sizeof(H));
}
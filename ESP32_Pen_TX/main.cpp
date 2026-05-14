/**
 * @file transmitter.ino
 * @brief Module Bút Laser (Transmitter) - ESP32
 *
 * CHỨC NĂNG:
 * - Đọc tín hiệu từ Rotary Encoder và 5 nút nhấn (Debounce logic).
 * - Quản lý nguồn thông minh: Chuyển đổi giữa Active, Light Sleep, Deep Sleep.
 * - Truyền dữ liệu: Giao thức ESP-NOW (độ trễ thấp, không cần bắt tay WiFi).
 *
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

// ===================== ĐỊNH DANH & TIMING =====================
#define DEBOUNCE_MS 25
#define SEND_TIMEOUT_MS 5 
#define SCROLL_LIMIT 5
#define LED_PULSE_MS 50
#define BATTERY_SEND_INTERVAL_MS 60000UL // Gửi pin mỗi 60 giây dù không có hoạt động
#define LIGHT_SLEEP_MS 300000UL          // 5 phút idle → light sleep
#define DEEP_SLEEP_MS 1800000UL          // 30 phút idle → deep sleep

// ===================== PIN GPIO =====================
#define LED_PIN 5
#define BTN_LEFT 4
#define BTN_RIGHT 13
#define BTN_NEXT 14
#define BTN_PREV 27
#define BTN_CAL 25
#define ENC_A 32
#define ENC_B 33
#define BAT_PIN 34

// ===================== ĐO PIN =====================
// ADC raw tương ứng với pin 3.3V (cạn) và 4.2V (đầy) sau cầu phân áp
#define BAT_ADC_EMPTY 1985 // ADC raw khi Vbat = 3.3V
#define BAT_ADC_FULL 2605  // ADC raw khi Vbat = 4.2V

// ===================== PACKET =====================
typedef struct __attribute__((packed))
{
    int8_t scroll;
    uint8_t buttons;
    uint8_t battery;
    uint8_t state; // 0=active, 1=sleeping
} pen_packet_t;

// ===================== ĐỊA CHỈ S3 =====================
static uint8_t s_s3_addr[] = {0x44, 0x1B, 0xF6, 0x8C, 0x1A, 0x1C};

static esp_now_peer_info_t s_peer_info; // Lưu peer info để re-add sau wake

// ===================== ENCODER =====================
static volatile int8_t s_enc_delta = 0;
static volatile uint8_t s_enc_last = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Lookup table quadrature: index = [prev_A, prev_B, cur_A, cur_B]
static const int8_t ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encoderISR()
{
    uint8_t state = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
    uint8_t idx = (s_enc_last << 2) | state;
    int8_t move = ENC_TABLE[idx];
    if (move != 0)
    {
        portENTER_CRITICAL_ISR(&s_mux);
        s_enc_delta += move;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
    s_enc_last = state;
}

// ===================== ĐỌC NÚT BẤM (debounce) =====================
static uint8_t readButtons()
{
    static uint32_t t_debounce = 0;
    static uint8_t stable = 0;
    static uint8_t last_raw = 0;

    uint8_t raw = 0;
    if (!digitalRead(BTN_LEFT))
        raw |= 0x01;
    if (!digitalRead(BTN_RIGHT))
        raw |= 0x02;
    if (!digitalRead(BTN_NEXT))
        raw |= 0x04;
    if (!digitalRead(BTN_PREV))
        raw |= 0x08;
    if (!digitalRead(BTN_CAL))
        raw |= 0x10;

    if (raw != last_raw)
    {
        t_debounce = millis();
        last_raw = raw;
    }
    if (millis() - t_debounce > DEBOUNCE_MS)
        stable = raw;
    return stable;
}

// ===================== ĐỌC PIN =====================
static uint8_t readBatteryPercent()
{
    int raw = analogRead(BAT_PIN);
    int pct = map(raw, BAT_ADC_EMPTY, BAT_ADC_FULL, 0, 100);
    return (uint8_t)constrain(pct, 0, 100);
}

// ===================== KHỞI TẠO WIFI + ESP-NOW =====================
static bool initEspNow()
{
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return false;
    }

    if (esp_now_add_peer(&s_peer_info) != ESP_OK)
    {
        Serial.println("ESP-NOW add peer failed");
        return false;
    }
    return true;
}

// ===================== SETUP =====================
void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_NEXT, INPUT_PULLUP);
    pinMode(BTN_PREV, INPUT_PULLUP);
    pinMode(BTN_CAL, INPUT_PULLUP);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(BAT_PIN, INPUT);
    
    memset(&s_peer_info, 0, sizeof(s_peer_info));
    memcpy(s_peer_info.peer_addr, s_s3_addr, 6);
    s_peer_info.channel = 1;
    s_peer_info.encrypt = false;

    initEspNow();

    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    // Cấu hình wakeup cho light sleep (GPIO level low)
    gpio_wakeup_enable((gpio_num_t)BTN_LEFT, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BTN_RIGHT, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BTN_CAL, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)ENC_A, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // Cấu hình wakeup cho deep sleep (ext0, chỉ BTN_CAL)
    // Lưu ý: ext0 và gpio_wakeup KHÔNG conflict vì dùng ở 2 sleep mode khác nhau
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_CAL, 0);
}

// ===================== LOOP CHÍNH =====================
void loop()
{
    static uint8_t s_last_buttons = 0;
    static uint32_t s_last_led_ms = 0;
    static bool s_led_active = false;
    static uint32_t s_last_activity = millis();
    static uint32_t s_last_bat_ms = 0;
    static bool s_sleep_notified = false; 

    pen_packet_t pkt = {0};

    // Đọc encoder atomic
    portENTER_CRITICAL(&s_mux);
    pkt.scroll = s_enc_delta;
    s_enc_delta = 0;
    portEXIT_CRITICAL(&s_mux);

    pkt.scroll = (int8_t)constrain(pkt.scroll, -SCROLL_LIMIT, SCROLL_LIMIT);
    pkt.buttons = readButtons();

    // Cập nhật thời gian hoạt động
    if (pkt.scroll != 0 || pkt.buttons != 0)
    {
        s_last_activity = millis();
        s_sleep_notified = false; // Reset để có thể notify lại lần sau
    }

    // --- Gửi khi có thay đổi ---
    bool should_send = (pkt.scroll != 0 || pkt.buttons != s_last_buttons);

    // --- Gửi pin định kỳ ngay cả khi không có hoạt động ---
    if (millis() - s_last_bat_ms > BATTERY_SEND_INTERVAL_MS)
    {
        should_send = true;
        s_last_bat_ms = millis();
    }

    if (should_send)
    {
        pkt.battery = readBatteryPercent();
        pkt.state = 0; // active
        esp_now_send(s_s3_addr, (uint8_t *)&pkt, sizeof(pkt));
        s_last_buttons = pkt.buttons;

        digitalWrite(LED_PIN, HIGH);
        s_led_active = true;
        s_last_led_ms = millis();
    }

    // Tắt LED sau LED_PULSE_MS
    if (s_led_active && millis() - s_last_led_ms > LED_PULSE_MS)
    {
        digitalWrite(LED_PIN, LOW);
        s_led_active = false;
    }

    // ===================== QUẢN LÝ SLEEP =====================
    uint32_t idle = millis() - s_last_activity;

    if (idle > DEEP_SLEEP_MS)
    {
        // --- Deep sleep ---
        Serial.println("Deep sleep...");
        digitalWrite(LED_PIN, LOW);

        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
        // ext0 (BTN_CAL) vẫn active để wake từ deep sleep

        esp_deep_sleep_start();
        // Sau deep sleep: chip reset hoàn toàn, setup() sẽ chạy lại
    }
    else if (idle > LIGHT_SLEEP_MS)
    {
        // --- Light sleep ---
        if (!s_sleep_notified)
        {

            pkt.scroll = 0;
            pkt.buttons = 0;
            pkt.battery = readBatteryPercent();
            pkt.state = 1;
            esp_now_send(s_s3_addr, (uint8_t *)&pkt, sizeof(pkt));

            delay(100);
            s_sleep_notified = true;
        }

        Serial.println("Light sleep...");
        esp_wifi_stop();
        esp_light_sleep_start();
        // --- Tiếp tục tại đây sau khi wake ---

        esp_wifi_start();
        initEspNow();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_GPIO)
        {
            s_last_activity = millis();
            s_sleep_notified = false;
            Serial.println("Woke by GPIO");
        }
        else
        {
            Serial.printf("Woke by other: %d\n", cause);
            // Không reset activity → sẽ ngủ lại ngay sau vài vòng loop
        }
    }

    delay(SEND_TIMEOUT_MS);
}
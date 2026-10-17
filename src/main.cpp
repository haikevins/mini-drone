#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// #define ARM_BUTTON_PIN 6
// #define RESET_BUTTON_PIN 7

#define OLED_SDA_PIN 8
#define OLED_SCL_PIN 9
#define OLED_RESET_PIN -1

#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64

#define JOYSTICK1_VRX_PIN 3
#define JOYSTICK1_VRY_PIN 4
#define JOYSTICK1_SW_PIN 20

#define JOYSTICK2_VRX_PIN 0
#define JOYSTICK2_VRY_PIN 1
#define JOYSTICK2_SW_PIN 10

typedef struct
{
    uint32_t counter;
} heartbeat_data_packet_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_data_packet_t;

// MUST match flight/src/communication/espnow_protocol.h exactly.
static constexpr uint32_t FLIGHT_STATUS_MAGIC = 0x31545346u;
static constexpr uint8_t FLIGHT_STATUS_ARMED = 1u << 0;
static constexpr uint8_t FLIGHT_STATUS_FAILSAFE = 1u << 1;
static constexpr uint8_t FLIGHT_STATUS_HB_RECENT = 1u << 2;
static constexpr uint8_t FLIGHT_STATUS_CMD_RECENT = 1u << 3;
static constexpr uint8_t FLIGHT_STATUS_IMU_READY = 1u << 4;

typedef struct
{
    uint32_t magic;
    uint32_t uptime_ms;
    uint32_t command_age_ms;
    uint16_t motor_output_us;
    uint8_t flags;
    uint8_t reason;
} flight_status_packet_t;
static_assert(sizeof(flight_status_packet_t) == 16, "Flight status ABI mismatch");

typedef struct
{
    bool arm;
    bool reset;

    bool throttle_up;
    bool throttle_down;

    bool direction_forward;
    bool direction_backward;
    bool direction_left;
    bool direction_right;
} command_data_packet_t;

enum class ThrottleState
{
    UNKNOWN,
    NEUTRAL,
    UP,
    DOWN
};

enum class DirectionState
{
    UNKNOWN,
    NEUTRAL,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

static attitude_data_packet_t attitude_data;
static command_data_packet_t command_data;
static heartbeat_data_packet_t heartbeat_data;

static uint32_t last_arm_button_press_time = 0u;
static uint32_t last_reset_button_press_time = 0u;
static uint32_t last_joystick1_sw_press_time = 0u;
static uint32_t last_joystick2_sw_press_time = 0u;

static bool last_arm_button_state = HIGH;
static bool last_reset_button_state = HIGH;
static bool last_joystick1_sw_state = HIGH;
static bool last_joystick2_sw_state = HIGH;

static bool is_armed = false;
static bool is_reset = false;

static ThrottleState last_throttle_state = ThrottleState::UNKNOWN;
static DirectionState last_direction_state = DirectionState::UNKNOWN;

// replace with your drone's mac address
static constexpr uint8_t drone_address[] = {0x14, 0x63, 0x93, 0x6E, 0x72, 0xE0};

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
static bool dbg_oled_ok = false;

static constexpr uint32_t display_update_interval = 100u;
static uint32_t last_display_update_time = 0u;

// Diagnostics are printed only from loop(), never Wi-Fi callbacks.
static volatile uint32_t dbg_tx_ok = 0;
static volatile uint32_t dbg_tx_fail = 0;
static volatile uint32_t dbg_rx_att = 0;
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static flight_status_packet_t s_flight_status{};
static uint32_t s_flight_status_last_ms = 0;
static bool s_flight_status_seen = false;
static uint32_t s_flight_status_count = 0;
static constexpr uint32_t g_status_timeout_ms = 750u;
static constexpr uint32_t g_arm_ack_timeout_ms = 1500u;
static bool g_arm_ack_seen = false;
static bool g_rearm_required = false;
static uint32_t g_arm_request_time_ms = 0u;

static bool flight_status_fresh(flight_status_packet_t &out, uint32_t &age_ms)
{
    uint32_t last_ms = 0;
    bool seen = false;

    portENTER_CRITICAL(&s_status_mux);
    out = s_flight_status;
    last_ms = s_flight_status_last_ms;
    seen = (s_flight_status_seen == true);
    portEXIT_CRITICAL(&s_status_mux);

    age_ms = millis() - last_ms;

    return ((seen == true) && (age_ms <= g_status_timeout_ms) && (out.magic == FLIGHT_STATUS_MAGIC));
}

static const char *status_reason_name(uint8_t reason)
{
    switch (reason)
    {
        case 0:
        {
            return "OK";
        }

        case 1:
        {
            return "WAIT_DISARM";
        }

        case 2:
        {
            return "STICK_NOT_NEUTRAL";
        }

        case 3:
        {
            return "CMD_TIMEOUT";
        }

        case 4:
        {
            return "HB_TIMEOUT";
        }

        case 5:
        {
            return "IMU_TIMEOUT";
        }

        default:
        {
            return "UNKNOWN";
        }
    }
}

// Flight owns the actual ARM state. Never continue sending an ARM request
// after confirmed Flight failsafe/disarm, invalid IMU or lost telemetry.
// A new physical ARM button press is required once Flight accepts DISARM.
void send_command();

static void update_flight_arm_sync(uint32_t now)
{
    if (is_armed == false)
    {
        g_arm_ack_seen = false;
        return;
    }

    flight_status_packet_t fc{};
    uint32_t age_ms = 0u;
    const bool status_ok = (flight_status_fresh(fc, age_ms) == true);
    const bool fc_armed = ((status_ok == true) && ((fc.flags & FLIGHT_STATUS_ARMED) != 0u));
    const bool fc_failsafe = ((status_ok == true) && ((fc.flags & FLIGHT_STATUS_FAILSAFE) != 0u));
    const bool imu_ready = ((status_ok == true) && ((fc.flags & FLIGHT_STATUS_IMU_READY) != 0u));
    const bool uplink_fresh =
        ((status_ok == true) && ((fc.flags & (FLIGHT_STATUS_CMD_RECENT | FLIGHT_STATUS_HB_RECENT)) ==
                                 (FLIGHT_STATUS_CMD_RECENT | FLIGHT_STATUS_HB_RECENT)));

    if ((fc_armed == true) && (imu_ready == true) && (fc_failsafe == false) && (uplink_fresh == true))
    {
        g_arm_ack_seen = true;
        return;
    }

    const bool ack_timed_out =
        ((g_arm_ack_seen == false) && (uint32_t(now - g_arm_request_time_ms) > g_arm_ack_timeout_ms));
    const bool remote_disarmed = ((g_arm_ack_seen == true) && (fc_armed == false));

    if ((status_ok == false) || (fc_failsafe == true) || (imu_ready == false) || (uplink_fresh == false) ||
        (ack_timed_out == true) || (remote_disarmed == true))
    {
        is_armed = false;
        g_arm_ack_seen = false;
        g_rearm_required = true;
        command_data = {};
        Serial.printf("[CTRL FAULT] Flight state lost/unsafe; DISARM sent, REARM_REQUIRED. "
                      "status=%u fc_arm=%u fs=%u imu=%u cmd_hb=%u ack_timeout=%u\n",
                      unsigned(status_ok),
                      unsigned(fc_armed),
                      unsigned(fc_failsafe),
                      unsigned(imu_ready),
                      unsigned(uplink_fresh),
                      unsigned(ack_timed_out));
        send_command();
    }
}

static volatile uint32_t dbg_rx_bad = 0;
static volatile uint32_t dbg_rx_last_ms = 0;
static volatile int dbg_rx_bad_len = 0;
static uint32_t dbg_command_queued = 0;
static uint32_t dbg_heartbeat_queued = 0;
static uint32_t dbg_send_api_failed = 0;
static uint32_t dbg_last_print_ms = 0;
static uint32_t dbg_last_command_retx_ms = 0;
static constexpr uint32_t dbg_print_period_ms = 500u;
static constexpr uint32_t dbg_command_retx_period_ms = 50u;

void send_heartbeat();

// Calibrate with Wi-Fi/ESP-NOW active: ADC behavior can change between builds.
// Calibrated controller; remote Flight independently enforces ARM and failsafe.
struct JoyCenter
{
    int j1x = 0, j1y = 0, j2x = 0, j2y = 0;
    bool valid = false;
};
static JoyCenter center;
static bool arm_neutral_ready = false;
static uint32_t arm_neutral_since = 0;
static uint32_t last_calib_warning = 0;

static bool near_value(int value, int target, int delta)
{
    return (abs(value - target) <= delta);
}

static int high_threshold(int calibrated_center)
{
    return calibrated_center + (4095 - calibrated_center) * 65 / 100;
}

static int low_threshold(int calibrated_center)
{
    return calibrated_center / 2;
}

// Some ADC reads can occasionally spike when Wi-Fi is active. The median of
// three reduces one-sample spikes without hiding a genuine joystick movement.
static int read_adc_filtered(uint8_t pin)
{
    int a = analogRead(pin);
    int b = analogRead(pin);
    int c = analogRead(pin);
    if (a > b)
    {
        int t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        int t = b;
        b = c;
        c = t;
    }

    if (a > b)
    {
        int t = a;
        a = b;
        b = t;
    }

    return b;
}

// Non-blocking calibration: radio packets, OLED and Flight DISARM commands
// continue to run while we gather readings. If the initial calibration fails,
// keep retrying while the controller is DISARMED instead of getting stuck in
// THR UNKNOWN / DIR UNKNOWN for the rest of the power cycle.
static constexpr uint8_t k_cal_samples = 64;
static constexpr uint32_t k_cal_sample_period_ms = 25u; // 1.6 s/window
static constexpr int k_cal_center_min = 3000;
static constexpr int k_cal_center_max = 3920;
static constexpr int k_cal_p10_p90_limit = 140;
static uint16_t s_cal_adc[4][k_cal_samples] = {};
static uint8_t s_cal_count = 0;
static uint32_t s_cal_last_sample_ms = 0;
static uint32_t s_cal_attempt = 0;

static void update_joystick_calibration(uint32_t now, int x1, int y1, int x2, int y2)
{
    if ((center.valid == true) || (is_armed == true))
    {
        return;
    }

    if (now - s_cal_last_sample_ms < k_cal_sample_period_ms)
    {
        return;
    }

    s_cal_last_sample_ms = now;
    if (s_cal_count == 0)
    {
        ++s_cal_attempt;
        Serial.printf("[CAL START] attempt=%lu; leave BOTH joysticks at rest...\n", (unsigned long)s_cal_attempt);
    }

    const int raw[4] = {x1, y1, x2, y2};
    for (int j = 0; j < 4; ++j)
    {
        s_cal_adc[j][s_cal_count] = (uint16_t)raw[j];
    }

    if (++s_cal_count < k_cal_samples)
    {
        return;
    }

    s_cal_count = 0;

    int med[4] = {};
    int spread[4] = {};
    bool stable = true;
    for (int axis = 0; axis < 4; ++axis)
    {
        uint16_t sorted[k_cal_samples];
        for (int i = 0; i < k_cal_samples; ++i)
        {
            sorted[i] = s_cal_adc[axis][i];
        }

        // Insertion sort; bounded input, no heap allocation required.
        for (int i = 1; i < k_cal_samples; ++i)
        {
            const uint16_t v = sorted[i];
            int j = i - 1;
            while ((j >= 0) && (sorted[j] > v))
            {
                sorted[j + 1] = sorted[j];
                --j;
            }

            sorted[j + 1] = v;
        }

        med[axis] = (int(sorted[31]) + int(sorted[32])) / 2;
        spread[axis] = int(sorted[57]) - int(sorted[6]); // 10th--90th pct.
        if ((med[axis] < k_cal_center_min) || (med[axis] > k_cal_center_max) || (spread[axis] > k_cal_p10_p90_limit))
        {
            stable = false;
        }
    }

    center.j1x = med[0];
    center.j1y = med[1];
    center.j2x = med[2];
    center.j2y = med[3];
    Serial.printf("[CAL CENTER] J1 x=%d y=%d | J2 x=%d y=%d\n", center.j1x, center.j1y, center.j2x, center.j2y);
    Serial.printf("[CAL SPREAD P10-P90] J1 x=%d y=%d | J2 x=%d y=%d (limit=%d)\n",
                  spread[0],
                  spread[1],
                  spread[2],
                  spread[3],
                  k_cal_p10_p90_limit);

    center.valid = (stable == true);
    if (center.valid == false)
    {
        Serial.println(
            "[CAL RETRY] ADC center/noise not safe; ARM BLOCKED, automatically retrying. Release both sticks.");
    }
    else
    {
        // A newly valid center does not ARM the drone. The user must still
        // return to neutral and make a NEW ARM button press.
        arm_neutral_since = 0;
        arm_neutral_ready = false;
        Serial.printf("[CAL OK] J1 UP>=%d DOWN<=%d | J2 F>=%d B<=%d R>=%d L<=%d\n",
                      high_threshold(center.j1x),
                      low_threshold(center.j1x),
                      high_threshold(center.j2x),
                      low_threshold(center.j2x),
                      high_threshold(center.j2y),
                      low_threshold(center.j2y));
    }
}

ThrottleState read_throttle_state(int xValue, int yValue);
DirectionState read_direction_state(int xValue, int yValue);

static const char *throttle_state_name(ThrottleState s)
{
    switch (s)
    {
        case ThrottleState::UP:
        {
            return "UP";
        }

        case ThrottleState::DOWN:
        {
            return "DOWN";
        }

        case ThrottleState::NEUTRAL:
        {
            return "NEUTRAL";
        }

        default:
        {
            return "CAL_WAIT";
        }
    }
}

static const char *direction_state_name(DirectionState s)
{
    switch (s)
    {
        case DirectionState::FORWARD:
        {
            return "FORWARD";
        }

        case DirectionState::BACKWARD:
        {
            return "BACKWARD";
        }

        case DirectionState::LEFT:
        {
            return "LEFT";
        }

        case DirectionState::RIGHT:
        {
            return "RIGHT";
        }

        case DirectionState::NEUTRAL:
        {
            return "NEUTRAL";
        }

        default:
        {
            return "CAL_WAIT";
        }
    }
}

static void debug_print_state(int x1, int y1, int x2, int y2)
{
    const uint32_t now = millis();
    const uint32_t age = (dbg_rx_att != 0u) ? now - dbg_rx_last_ms : 0u;
    uint8_t ch = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&ch, &second);

    Serial.printf("[CTRL ADC] J1 x=%d y=%d =>%s | J2 x=%d y=%d =>%s\n",
                  x1,
                  y1,
                  throttle_state_name(read_throttle_state(x1, y1)),
                  x2,
                  y2,
                  direction_state_name(read_direction_state(x2, y2)));
    Serial.printf("[CTRL STATE] local_arm=%u cmd: reset=%u UP=%u DOWN=%u F=%u B=%u L=%u R=%u wifi_ch=%u\n",
                  unsigned(is_armed),
                  unsigned(command_data.reset),
                  unsigned(command_data.throttle_up),
                  unsigned(command_data.throttle_down),
                  unsigned(command_data.direction_forward),
                  unsigned(command_data.direction_backward),
                  unsigned(command_data.direction_left),
                  unsigned(command_data.direction_right),
                  unsigned(ch));

    Serial.printf(
        "[CTRL SYNC] fc_ack_seen=%u rearm_required=%u\n", unsigned(g_arm_ack_seen), unsigned(g_rearm_required));
    Serial.printf("[CTRL LINK] cmd_queue=%lu hb_queue=%lu send_api_err=%lu mac_delivery_ok=%lu mac_delivery_fail=%lu ",
                  (unsigned long)dbg_command_queued,
                  (unsigned long)dbg_heartbeat_queued,
                  (unsigned long)dbg_send_api_failed,
                  (unsigned long)dbg_tx_ok,
                  (unsigned long)dbg_tx_fail);

    Serial.printf("rx_telemetry=%lu age_ms=", (unsigned long)dbg_rx_att);
    if (dbg_rx_att != 0u)
    {
        Serial.print((unsigned long)age);
    }
    else
    {
        Serial.print("NEVER");
    }

    Serial.printf(" rx_invalid=%lu last_invalid_len=%d\n", (unsigned long)dbg_rx_bad, (int)dbg_rx_bad_len);
    if (dbg_rx_att != 0u)
    {
        Serial.printf("[CTRL TELEMETRY] roll=%.1f pitch=%.1f yaw=%.1f\n",
                      attitude_data.roll,
                      attitude_data.pitch,
                      attitude_data.yaw);
    }

    flight_status_packet_t st{};
    uint32_t age_ms = 0;
    const bool recent = (flight_status_fresh(st, age_ms) == true);
    Serial.printf("[CTRL FC] status_recent=%u age_ms=%lu fc_arm=%u failsafe=%u cmd_recent=%u hb_recent=%u motor_us=%u "
                  "reason=%s\n",
                  unsigned(recent),
                  (unsigned long)age_ms,
                  unsigned(((recent == true) && ((st.flags & FLIGHT_STATUS_ARMED) != 0u))),
                  unsigned(((recent == true) && ((st.flags & FLIGHT_STATUS_FAILSAFE) != 0u))),
                  unsigned(((recent == true) && ((st.flags & FLIGHT_STATUS_CMD_RECENT) != 0u))),
                  unsigned(((recent == true) && ((st.flags & FLIGHT_STATUS_HB_RECENT) != 0u))),
                  unsigned((recent == true) ? st.motor_output_us : 0u),
                  (recent == true) ? status_reason_name(st.reason) : "NO_STATUS");
}

static constexpr uint32_t heartbeat_update_interval = 100u;
static uint32_t last_heartbeat_data_send_time = 0u;

void send_command()
{
    esp_err_t result = esp_now_send(drone_address, reinterpret_cast<uint8_t *>(&command_data), sizeof(command_data));

    if (result == ESP_OK)
    {
        dbg_command_queued++;
    }
    else
    {
        dbg_send_api_failed++;
        Serial.printf("[CTRL TX CMD API ERROR] esp_err_t=%d\n", int(result));
    }
}

void send_heartbeat()
{
    heartbeat_data.counter++;

    esp_err_t result =
        esp_now_send(drone_address, reinterpret_cast<uint8_t *>(&heartbeat_data), sizeof(heartbeat_data));

    if (result == ESP_OK)
    {
        dbg_heartbeat_queued++;
    }
    else
    {
        dbg_send_api_failed++;
        Serial.printf("[CTRL TX HB API ERROR] esp_err_t=%d\n", int(result));
    }
}

ThrottleState read_throttle_state(int xValue, int yValue)
{
    (void)yValue; // J1-Y is unused by the original controller.
    if (center.valid == false)
    {
        return ThrottleState::UNKNOWN;
    }

    if (xValue >= high_threshold(center.j1x))
    {
        return ThrottleState::UP;
    }

    if (xValue <= low_threshold(center.j1x))
    {
        return ThrottleState::DOWN;
    }

    return ThrottleState::NEUTRAL;
}

DirectionState read_direction_state(int xValue, int yValue)
{
    if (center.valid == false)
    {
        return DirectionState::UNKNOWN;
    }

    bool forward = (xValue >= high_threshold(center.j2x));
    bool backward = (xValue <= low_threshold(center.j2x));
    bool right = (yValue >= high_threshold(center.j2y));
    bool left = (yValue <= low_threshold(center.j2y));

    // A diagonal beyond both thresholds is deliberately treated as neutral.
    if (int(forward) + int(backward) + int(right) + int(left) != 1)
    {
        return DirectionState::NEUTRAL;
    }

    if (forward == true)
    {
        return DirectionState::FORWARD;
    }

    if (backward == true)
    {
        return DirectionState::BACKWARD;
    }

    if (right == true)
    {
        return DirectionState::RIGHT;
    }

    return DirectionState::LEFT;
}

void apply_throttle_state_to_command(ThrottleState throttle_state)
{
    switch (throttle_state)
    {
        case ThrottleState::UP:
        {
            command_data.throttle_up = true;
            command_data.throttle_down = false;
            break;
        }

        case ThrottleState::DOWN:
        {
            command_data.throttle_up = false;
            command_data.throttle_down = true;
            break;
        }

        case ThrottleState::NEUTRAL:
        {
            command_data.throttle_up = false;
            command_data.throttle_down = false;
            break;
        }

        case ThrottleState::UNKNOWN:
        {
            // Intentional fall through to the next case.
        }

        default:
        {
            command_data.throttle_up = false;
            command_data.throttle_down = false;
            break;
        }
    }
}

void apply_direction_state_to_command(DirectionState direction_state)
{
    switch (direction_state)
    {
        case DirectionState::FORWARD:
        {
            command_data.direction_forward = true;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;
        }

        case DirectionState::BACKWARD:
        {
            command_data.direction_forward = false;
            command_data.direction_backward = true;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;
        }

        case DirectionState::LEFT:
        {
            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = true;
            command_data.direction_right = false;
            break;
        }

        case DirectionState::RIGHT:
        {
            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = true;
            break;
        }

        case DirectionState::NEUTRAL:
        {
            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;
        }

        case DirectionState::UNKNOWN:
        {
            // Intentional fall through to the next case.
        }

        default:
        {
            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;
        }
    }
}

void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if ((info == nullptr) || (info->src_addr == nullptr) || (memcmp(info->src_addr, drone_address, 6) != 0) ||
        (data == nullptr))
    {
        return;
    }

    if (len == int(sizeof(flight_status_packet_t)))
    {
        flight_status_packet_t st{};
        memcpy(&st, data, sizeof(st));
        if (st.magic != FLIGHT_STATUS_MAGIC)
        {
            dbg_rx_bad++;
            dbg_rx_bad_len = len;
            return;
        }

        portENTER_CRITICAL(&s_status_mux);
        s_flight_status = st;
        s_flight_status_last_ms = millis();
        s_flight_status_seen = true;
        s_flight_status_count++;
        portEXIT_CRITICAL(&s_status_mux);
        return;
    }

    if (len == int(sizeof(attitude_data_packet_t)))
    {
        memcpy(&attitude_data, data, sizeof(attitude_data));
        dbg_rx_att++;
        dbg_rx_last_ms = millis();
        return;
    }

    dbg_rx_bad++;
    dbg_rx_bad_len = len;
}

void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        dbg_tx_ok++;
    }
    else
    {
        dbg_tx_fail++;
    }

    // Serial.print("\nLast Packet Send Status: ");

    // if (status == ESP_NOW_SEND_SUCCESS)
    // {
    //     Serial.println("Delivery Success");
    // }
    // else
    // {
    //     Serial.println("Delivery Fail");
    // }
}

void setup()
{
    Serial.begin(115200);
    const uint32_t serial_start_ms = millis();
    while ((static_cast<bool>(Serial) == false) && (millis() - serial_start_ms < 1500u))
    {
        delay(10);
    }

    delay(250);
    Serial.println("[CTRL] ESP-NOW controller started");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.printf("[CTRL WIFI] local_sta_mac=%s expected_flight_mac=%02X:%02X:%02X:%02X:%02X:%02X channel=%d\n",
                  WiFi.macAddress().c_str(),
                  drone_address[0],
                  drone_address[1],
                  drone_address[2],
                  drone_address[3],
                  drone_address[4],
                  drone_address[5],
                  WiFi.channel());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        for (;;)
        {
        }
    }

    Serial.println("[CTRL ESPNOW] init OK");
    esp_now_register_recv_cb(on_data_recv);
    esp_now_register_send_cb(on_data_sent);

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, drone_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        for (;;)
        {
        }
    }

    Serial.println("[CTRL ESPNOW] peer registered OK");

    // pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);
    // pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

    pinMode(JOYSTICK1_SW_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_SW_PIN, INPUT_PULLUP);

    analogReadResolution(12);

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

    dbg_oled_ok = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (dbg_oled_ok == false)
    {
        Serial.println(F("[CTRL OLED] SSD1306 failed: continue with Serial only"));
    }
    else
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
    }

    delay(600);
    Serial.println("[CAL] Auto-calibration runs in loop, without blocking ESPNOW commands.");
    Serial.println("[CAL] Keep BOTH joysticks centered; ARM stays blocked until [CAL OK].");
}

void loop()
{
    const uint32_t now = millis();
    const bool arm_button = (static_cast<bool>(digitalRead(JOYSTICK1_SW_PIN)) == true);
    const bool reset_button = (static_cast<bool>(digitalRead(JOYSTICK2_SW_PIN)) == true);
    const int x1 = read_adc_filtered(JOYSTICK1_VRX_PIN);
    const int y1 = read_adc_filtered(JOYSTICK1_VRY_PIN);
    const int x2 = read_adc_filtered(JOYSTICK2_VRX_PIN);
    const int y2 = read_adc_filtered(JOYSTICK2_VRY_PIN);

    update_joystick_calibration(now, x1, y1, x2, y2);

    ThrottleState throttle_state = read_throttle_state(x1, y1);
    DirectionState direction_state = read_direction_state(x2, y2);

    const bool is_neutral = ((center.valid == true) && (throttle_state == ThrottleState::NEUTRAL) &&
                             (direction_state == DirectionState::NEUTRAL) &&
                             (near_value(y1, center.j1y, 250) == true) && (near_value(x1, center.j1x, 80) == true) &&
                             (near_value(x2, center.j2x, 110) == true) && (near_value(y2, center.j2y, 110) == true));

    if (is_neutral == true)
    {
        if (arm_neutral_since == 0u)
        {
            arm_neutral_since = now;
        }
    }
    else
    {
        arm_neutral_since = 0;
    }

    arm_neutral_ready = ((is_neutral == true) && (now - arm_neutral_since >= 700));

    update_flight_arm_sync(now);

    if ((last_joystick1_sw_state == HIGH) && (arm_button == LOW) && (now - last_joystick1_sw_press_time > 250))
    {
        last_joystick1_sw_press_time = now;
        if (is_armed == true)
        {
            is_armed = false;
            g_arm_ack_seen = false;
            g_rearm_required = false;
            Serial.println("[CTRL ARM] DISARM (local command)");
        }
        else
        {
            flight_status_packet_t fc{};
            uint32_t fc_age_ms = 0;
            const bool status_ok = (flight_status_fresh(fc, fc_age_ms) == true);
            const bool remote_ready =
                ((status_ok == true) && ((fc.flags & FLIGHT_STATUS_HB_RECENT) != 0u) &&
                 ((fc.flags & FLIGHT_STATUS_CMD_RECENT) != 0u) && ((fc.flags & FLIGHT_STATUS_ARMED) == 0u) &&
                 ((fc.flags & FLIGHT_STATUS_FAILSAFE) == 0u) && ((fc.flags & FLIGHT_STATUS_IMU_READY) != 0u));
            if ((arm_neutral_ready == true) && (remote_ready == true))
            {
                is_armed = true;
                g_arm_ack_seen = false;
                g_rearm_required = false;
                g_arm_request_time_ms = now;
                Serial.println("[CTRL ARM REQUEST] ARM=1 sent; wait for [CTRL FC] fc_arm=1 ACK");
            }
            else
            {
                is_armed = false;
                Serial.printf("[CTRL ARM BLOCKED] stick_ready=%u fc_ready=%u status_recent=%u fc_reason=%s\n",
                              unsigned(arm_neutral_ready),
                              unsigned(remote_ready),
                              unsigned(status_ok),
                              (status_ok == true) ? status_reason_name(fc.reason) : "NO_STATUS");
            }
        }

        command_data.arm = (is_armed == true);
        command_data.reset = false;

        // Clear everything immediately before the first state packet.
        command_data.throttle_up = false;
        command_data.throttle_down = false;
        command_data.direction_forward = false;
        command_data.direction_backward = false;
        command_data.direction_left = false;
        command_data.direction_right = false;
        send_command();
    }

    last_joystick1_sw_state = (arm_button == true);

    if ((last_joystick2_sw_state == HIGH) && (reset_button == LOW) && (now - last_joystick2_sw_press_time > 250))
    {
        last_joystick2_sw_press_time = now;
        is_armed = false;
        g_arm_ack_seen = false;
        g_rearm_required = false;
        command_data = {};
        command_data.reset = true;
        send_command();
        command_data.reset = false;
        Serial.println("[CTRL RESET] Reset command sent; local DISARM");
    }

    last_joystick2_sw_state = (reset_button == true);

    // Rebuild the complete command every cycle. Never retain a stale direction.
    command_data.arm = (is_armed == true);
    command_data.reset = false;
    if ((is_armed == true) && (g_arm_ack_seen == true) && (center.valid == true))
    {
        apply_throttle_state_to_command(throttle_state);
        apply_direction_state_to_command(direction_state);
    }
    else
    {
        command_data.throttle_up = false;
        command_data.throttle_down = false;
        command_data.direction_forward = false;
        command_data.direction_backward = false;
        command_data.direction_left = false;
        command_data.direction_right = false;
    }

    if (now - last_heartbeat_data_send_time >= heartbeat_update_interval)
    {
        last_heartbeat_data_send_time = now;
        send_heartbeat();
    }

    if (now - dbg_last_command_retx_ms >= dbg_command_retx_period_ms)
    {
        dbg_last_command_retx_ms = now;
        send_command();
    }

    if ((dbg_oled_ok == true) && (now - last_display_update_time >= display_update_interval))
    {
        last_display_update_time = now;
        display.clearDisplay();
        display.setCursor(0, 0);
        flight_status_packet_t fc{};
        uint32_t age_ms = 0;
        const bool fc_recent = (flight_status_fresh(fc, age_ms) == true);
        display.print("ARM TX: ");
        display.println((is_armed == true) ? "ON" : "OFF");
        display.setCursor(0, 10);
        display.print("ARM FC: ");
        if (fc_recent == false)
        {
            display.println("NO STATUS");
        }
        else
        {
            display.println(((fc.flags & FLIGHT_STATUS_ARMED) != 0u) ? "ON" : "OFF");
        }

        display.setCursor(0, 20);
        const bool uplink_ok =
            ((fc_recent == true) && ((fc.flags & (FLIGHT_STATUS_HB_RECENT | FLIGHT_STATUS_CMD_RECENT)) ==
                                     (FLIGHT_STATUS_HB_RECENT | FLIGHT_STATUS_CMD_RECENT)));
        display.print("LINK: ");
        display.print((uplink_ok == true) ? "OK" : "LOST");
        display.print(" FS:");
        if (fc_recent == false)
        {
            display.println("--");
        }
        else
        {
            display.println(((fc.flags & FLIGHT_STATUS_FAILSAFE) != 0u) ? "YES" : "NO");
        }

        display.setCursor(0, 30);
        display.print("MOTOR: ");
        if (fc_recent == true)
        {
            display.println(fc.motor_output_us);
        }
        else
        {
            display.println("--");
        }

        display.setCursor(0, 40);
        display.print("THR: ");
        display.println(throttle_state_name(throttle_state));
        display.setCursor(0, 50);
        if (g_rearm_required == true)
        {
            display.println("REARM: PRESS ARM");
        }
        else if ((is_armed == true) && (g_arm_ack_seen == false))
        {
            display.println("ARM: WAIT FC ACK");
        }
        else
        {
            display.print("DIR: ");
            display.println(direction_state_name(direction_state));
        }

        display.display();
    }

    delay(10);
}

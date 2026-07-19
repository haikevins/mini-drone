#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
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
#define JOYSTICK1_SW_PIN  20

#define JOYSTICK2_VRX_PIN 0
#define JOYSTICK2_VRY_PIN 1
#define JOYSTICK2_SW_PIN  10

typedef struct
{
    uint32_t counter;
}
heartbeat_data_packet_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}
attitude_data_packet_t;

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
}
command_data_packet_t;

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
static constexpr uint8_t drone_address[] = {0xEC, 0xDA, 0x3B, 0xBF, 0x7B, 0xC4};

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

static constexpr uint32_t display_update_interval = 100u;
static uint32_t last_display_update_time = 0u;

static constexpr uint32_t heartbeat_update_interval = 100u;
static uint32_t last_heartbeat_data_send_time = 0u;

void send_command()
{
    esp_err_t result = esp_now_send(
        drone_address,
        reinterpret_cast<uint8_t *>(&command_data),
        sizeof(command_data)
    );

    if (result != ESP_OK)
    {
        Serial.println("ESP-NOW send failed");
    }
}

void send_heartbeat()
{
    heartbeat_data.counter++;

    esp_err_t result = esp_now_send(
        drone_address,
        reinterpret_cast<uint8_t *>(&heartbeat_data),
        sizeof(heartbeat_data)
    );

    if (result != ESP_OK)
    {
        Serial.println("ESP-NOW heartbeat send failed");
    }
}

ThrottleState read_throttle_state(int xValue, int yValue)
{
    if ((xValue < 3900 && xValue > 3500) &&
        (yValue < 3800 && yValue > 3500))
    {
        return ThrottleState::NEUTRAL;
    }

    if (yValue > 3600)
    {
        if (xValue > 4000)
        {
            return ThrottleState::UP;
        }
        else if (xValue < 30)
        {
            return ThrottleState::DOWN;
        }
    }

    return ThrottleState::UNKNOWN;
}

DirectionState read_direction_state(int xValue, int yValue)
{
    if ((xValue < 3900 && xValue > 3500) &&
        (yValue < 3800 && yValue > 3500))
    {
        return DirectionState::NEUTRAL;
    }

    if (yValue > 3600)
    {
        if (xValue > 4000)
        {
            return DirectionState::FORWARD;
        }
        else if (xValue < 30)
        {
            return DirectionState::BACKWARD;
        }
    }

    if (xValue > 3600)
    {
        if (yValue > 4000)
        {
            return DirectionState::RIGHT;
        }
        else if (yValue < 30)
        {
            return DirectionState::LEFT;
        }
    }

    return DirectionState::UNKNOWN;
}

void apply_throttle_state_to_command(ThrottleState throttle_state)
{
    switch (throttle_state)
    {
        case ThrottleState::UP:
            Serial.println("Throttle Up");

            command_data.throttle_up = true;
            command_data.throttle_down = false;
            break;

        case ThrottleState::DOWN:
            Serial.println("Throttle Down");

            command_data.throttle_up = false;
            command_data.throttle_down = true;
            break;

        case ThrottleState::NEUTRAL:
            Serial.println("Throttle Neutral");

            command_data.throttle_up = false;
            command_data.throttle_down = false;
            break;

        case ThrottleState::UNKNOWN:
        default:
            break;
    }
}

void apply_direction_state_to_command(DirectionState direction_state)
{
    switch (direction_state)
    {
        case DirectionState::FORWARD:
            Serial.println("Direction Forward");

            command_data.direction_forward = true;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;

        case DirectionState::BACKWARD:
            Serial.println("Direction Backward");

            command_data.direction_forward = false;
            command_data.direction_backward = true;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;

        case DirectionState::LEFT:
            Serial.println("Direction Left");

            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = true;
            command_data.direction_right = false;
            break;

        case DirectionState::RIGHT:
            Serial.println("Direction Right");

            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = true;
            break;

        case DirectionState::NEUTRAL:
            Serial.println("Direction Neutral");

            command_data.direction_forward = false;
            command_data.direction_backward = false;
            command_data.direction_left = false;
            command_data.direction_right = false;
            break;

        case DirectionState::UNKNOWN:
        default:
            break;
    }
}

void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;

    if (data == nullptr || len != sizeof(attitude_data_packet_t))
    {
        Serial.println("Invalid attitude packet");
        return;
    }

    memcpy(&attitude_data, data, sizeof(attitude_data));

    // Serial.print("\r\nBytes received: ");
    // Serial.println(len);
    // Serial.print("Roll: ");
    // Serial.println(attitude_data.roll);
    // Serial.print("Pitch: ");
    // Serial.println(attitude_data.pitch);
    // Serial.print("Yaw: ");
    // Serial.println(attitude_data.yaw);
    // Serial.println();
}

void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    (void)status;

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
    // Serial.begin(115200);
    // delay(2000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error initializing ESP-NOW");
        for(;;) {}
    }

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
        for(;;) {}
    }
    
    // pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);
    // pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

    pinMode(JOYSTICK1_SW_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_SW_PIN, INPUT_PULLUP);

    analogReadResolution(12);

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
    {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;) {}
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
}

void loop() 
{
    const uint32_t now = millis();

    // bool current_arm_button_state = digitalRead(ARM_BUTTON_PIN);
    // bool current_reset_button_state = digitalRead(RESET_BUTTON_PIN);
    bool current_joystick1_sw_state = digitalRead(JOYSTICK1_SW_PIN);
    bool current_joystick2_sw_state = digitalRead(JOYSTICK2_SW_PIN);

    int xValue1 = analogRead(JOYSTICK1_VRX_PIN);
    int yValue1 = analogRead(JOYSTICK1_VRY_PIN);

    int xValue2 = analogRead(JOYSTICK2_VRX_PIN);
    int yValue2 = analogRead(JOYSTICK2_VRY_PIN);

    // handle arm button
    if ((last_joystick1_sw_state == HIGH) &&
        (current_joystick1_sw_state == LOW) &&
        ((now - last_joystick1_sw_press_time) > 200))
    {
        last_joystick1_sw_press_time = now;

        is_armed = !is_armed;

        command_data.arm = is_armed;
        command_data.reset = false;
        command_data.throttle_up = false;
        command_data.throttle_down = false;

        last_throttle_state = ThrottleState::UNKNOWN;

        if (is_armed == true)
        {
            Serial.println("Button Pressed Arm");
        }
        else
        {
            Serial.println("Button Pressed Disarm");
        }

        send_command();
    }

    last_joystick1_sw_state = current_joystick1_sw_state;

    // Handle reset button
    if ((last_joystick2_sw_state == HIGH) &&
        (current_joystick2_sw_state == LOW) &&
        ((now - last_joystick2_sw_press_time) > 200))
    {
        last_joystick2_sw_press_time = now;

        Serial.println("Button Pressed Reset");

        is_armed = false;

        command_data.arm = false;
        command_data.reset = true;
        command_data.throttle_up = false;
        command_data.throttle_down = false;

        last_throttle_state = ThrottleState::UNKNOWN;

        send_command();

        command_data.arm = false;
        command_data.reset = false;
        command_data.throttle_up = false;
        command_data.throttle_down = false;
    }

    last_joystick2_sw_state = current_joystick2_sw_state;

    if (is_armed == true)
    {
        ThrottleState current_throttle_state = read_throttle_state(xValue1, yValue1);

        if (current_throttle_state != ThrottleState::UNKNOWN &&
            current_throttle_state != last_throttle_state)
        {
            command_data.arm = true;
            command_data.reset = false;

            apply_throttle_state_to_command(current_throttle_state);

            send_command();

            last_throttle_state = current_throttle_state;
        }

        DirectionState current_direction_state = read_direction_state(xValue2, yValue2);

        if (current_direction_state != DirectionState::UNKNOWN &&
            current_direction_state != last_direction_state)
        {
            command_data.arm = true;
            command_data.reset = false;

            apply_direction_state_to_command(current_direction_state);

            send_command();

            last_direction_state = current_direction_state;
        }
    }

    if (now - last_heartbeat_data_send_time >= heartbeat_update_interval)
    {
        last_heartbeat_data_send_time = now;
        send_heartbeat();
    }

    if (now - last_display_update_time >= display_update_interval)
    {
        last_display_update_time = now;

        display.clearDisplay();

        display.setCursor(0, 0);
        display.print("Arm: ");
        display.println(is_armed ? "ON" : "OFF");

        display.setCursor(0, 10);
        display.print("Throttle: ");
        display.println(command_data.throttle_up ? "UP" : (command_data.throttle_down ? "DOWN" : "NEUTRAL"));

        display.setCursor(0, 20);
        display.print("Direction: ");
        if (command_data.direction_forward)
        {
            display.println("FORWARD");
        }
        else if (command_data.direction_backward)
        {
            display.println("BACKWARD");
        }
        else if (command_data.direction_left)
        {
            display.println("LEFT");
        }
        else if (command_data.direction_right)
        {
            display.println("RIGHT");
        }
        else
        {
            display.println("NEUTRAL");
        }

        display.setCursor(0, 30);
        display.print("Roll: ");
        display.println(attitude_data.roll);

        display.setCursor(0, 40);
        display.print("Pitch: ");
        display.println(attitude_data.pitch);

        display.setCursor(0, 50);
        display.print("Yaw: ");
        display.println(attitude_data.yaw);

        display.display();
    }
}
/*
 * main.cpp
 */

#include <Arduino.h>
#include <math.h>

#include "drivers/mpu6500.h"
#include "estimator/madgwick.h"

#include "control/pid_controller.h"

#include "communication/espnow.h"
#include "communication/espnow_protocol.h"

#include "hal/motors.h"

static SPIBus g_spi_bus;
static MPU6500 g_imu;
static Madgwick g_madgwick;

static Motors g_motors;

// kp, ki, kd, integLimit, outputLimit, dFilterAlpha
static PIDController g_pid_roll_angle(5.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f); 
static PIDController g_pid_pitch_angle(5.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f);

static PIDController g_pid_roll_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_pitch_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_yaw_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);

static ESPNow g_espnow;

static attitude_data_packet_t g_attitude_data =
{
    0.0f,
    0.0f,
    0.0f
};

static constexpr float g_target_yaw_rate = 0.0f;
static float g_target_roll_angle = 0.0f;
static float g_target_pitch_angle = 0.0f;

static constexpr float g_direction_deg = 10.0f;

static constexpr uint8_t g_imu_pin_sck = 4u;
static constexpr uint8_t g_imu_pin_miso = 5u;
static constexpr uint8_t g_imu_pin_mosi = 6u;
static constexpr uint8_t g_imu_pin_ncs = 7u;

static constexpr float g_imu_read_default_hz = 1000.0f;

static constexpr float g_imu_gyro_lpf_alpha = 0.22f;
static constexpr float g_imu_accel_lpf_alpha = 0.10f;

static constexpr float g_madgwick_beta_disarm = 0.10f;
static constexpr float g_madgwick_beta_idle = 0.06f;
static constexpr float g_madgwick_beta_flying = 0.03f;
static constexpr float g_madgwick_beta_min = 0.003f;

static constexpr float g_madgwick_acc_error_good = 0.08f;
static constexpr float g_madgwick_acc_error_bad = 0.25f;

static constexpr float g_madgwick_beta_alpha = 0.02f;
static constexpr float g_madgwick_confidence_min = 0.90f;

static float g_madgwick_beta_current = g_madgwick_beta_disarm;

static constexpr uint32_t g_espnow_trans_period_ms = 100u; // 10 Hz
static uint32_t g_espnow_trans_last_time = 0u;

static constexpr uint32_t g_espnow_heartbeat_timeout_ms = 1000u; // 1000 ms

static constexpr float g_motor_throttle_base_min = 1000.0f;   // motor stop / disarmed
static constexpr float g_motor_throttle_base_max = 1400.0f;

static constexpr float g_motor_throttle_idle = 1100.0f;  // armed idle
static constexpr float g_motor_throttle_min = 1000.0f;  // min throttle
static constexpr float g_motor_throttle_max = 2000.0f;  // max throttle

static constexpr float g_throttle_up_step_per_second = 200.0f;
static constexpr float g_throttle_down_step_per_second = 60.0f;

static constexpr float g_failsafe_throttle_step_per_second = 60.0f;
static bool g_failsafe_active = false;

static float g_motor_throttle_base = g_motor_throttle_idle;
static bool g_was_armed = false;

static void setup_imu();
static void setup_pid();
static void setup_espnow();

static float accel_confidence(float ax, float ay, float az);
static float update_madgwick_beta(float ax, float ay, float az);

static void update_attitude_target();
static void reset_attitude_target();

void setup()
{
    // Serial.begin(115200);
    // delay(2000);

    setup_imu();
    setup_pid();
    setup_espnow();

    g_motors.setup_motors();
}

void loop()
{
    g_motors.write_all_motors(1100.0f);
    /*
    const uint32_t now = millis();

    if (g_imu.update() == false)
    {
        return;
    }

    const auto& imu_data = g_imu.get_filtered();
    const auto& imu_dt = g_imu.get_timing().dt;

    const float beta = update_madgwick_beta(
        imu_data.ax,
        imu_data.ay,
        imu_data.az
    );

    g_madgwick.setBeta(beta);

    g_madgwick.updateIMUdt(
        imu_data.gx,
        imu_data.gy,
        imu_data.gz,
        imu_data.ax,
        imu_data.ay,
        imu_data.az,
        imu_dt
    );

    g_attitude_data.roll = g_madgwick.getRoll();
    g_attitude_data.pitch = g_madgwick.getPitch();
    g_attitude_data.yaw = g_madgwick.getYaw();

    if (g_espnow.is_reset() == true) 
    { 
        g_motor_throttle_base = g_motor_throttle_idle; 
        g_motors.write_all_motors(g_motor_throttle_base_min);
        g_espnow.reset_command(); 
        
        delay(100); 
        esp_restart();
         
        return; 
    }

    const bool is_heartbeat_recent = g_espnow.is_heartbeat_recent(g_espnow_heartbeat_timeout_ms);
    if ((g_espnow.is_armed() == true) && (is_heartbeat_recent == false))
    {
        g_failsafe_active = true;
    }

    if ((g_espnow.is_armed() == true) && (g_failsafe_active == true))
    {
        g_motor_throttle_base -= g_failsafe_throttle_step_per_second * imu_dt;
        g_motor_throttle_base = max(g_motor_throttle_base, g_motor_throttle_idle);

        const bool apply_pid = (g_motor_throttle_base > (g_motor_throttle_idle + 1.0f));
        if (apply_pid == true)
        {
            // Angle PID: attitude angle error -> target angular rate.
            const float roll_rate_sp = g_pid_roll_angle.update(
                g_target_roll_angle,
                g_attitude_data.roll,
                imu_dt,
                false,
                true
            );

            const float pitch_rate_sp = g_pid_pitch_angle.update(
                g_target_pitch_angle,
                g_attitude_data.pitch,
                imu_dt,
                false,
                true
            );

            // Rate PID: target angular rate - gyro rate -> motor correction.
            float roll_output = g_pid_roll_rate.update(roll_rate_sp, imu_data.gx, imu_dt, false, true);
            float pitch_output = g_pid_pitch_rate.update(pitch_rate_sp, imu_data.gy, imu_dt, false, true);
            float yaw_output = g_pid_yaw_rate.update(g_target_yaw_rate, imu_data.gz, imu_dt, false, true);

            float motor1_speed = g_motor_throttle_base + roll_output - pitch_output + yaw_output; // front left
            float motor2_speed = g_motor_throttle_base - roll_output - pitch_output - yaw_output + 60.0f; // front right
            float motor3_speed = g_motor_throttle_base - roll_output + pitch_output + yaw_output + 120.0f; // rear right
            float motor4_speed = g_motor_throttle_base + roll_output + pitch_output - yaw_output + 10.0f; // rear left

            float max_motor_speed = max(max(motor1_speed, motor2_speed), max(motor3_speed, motor4_speed));
            if (max_motor_speed > g_motor_throttle_max)
            {
                float diff = max_motor_speed - g_motor_throttle_max;
                motor1_speed -= diff;
                motor2_speed -= diff;
                motor3_speed -= diff;
                motor4_speed -= diff;
            }

            float min_motor_speed = min(min(motor1_speed, motor2_speed), min(motor3_speed, motor4_speed));
            if (min_motor_speed < g_motor_throttle_min)
            {
                float diff = g_motor_throttle_min - min_motor_speed;
                motor1_speed += diff;
                motor2_speed += diff;
                motor3_speed += diff;
                motor4_speed += diff;
            }

            g_motors.write_motors(
                motor1_speed,
                motor2_speed,
                motor3_speed,
                motor4_speed
            );
        }
        else
        {
            setup_pid();
            g_motors.write_all_motors(g_motor_throttle_idle);
        }            
    }    
    else if ((g_espnow.is_armed() == true) && (g_failsafe_active == false))
    {
        if (g_was_armed == false)
        {
            g_motor_throttle_base = g_motor_throttle_idle;
            g_was_armed = true;
        }

        if (g_espnow.is_throttle_up() == true)
        {
            g_motor_throttle_base += g_throttle_up_step_per_second * imu_dt;
        }        
        else if (g_espnow.is_throttle_down() == true)
        {
            g_motor_throttle_base -= g_throttle_down_step_per_second * imu_dt;
        }

        g_motor_throttle_base = constrain(g_motor_throttle_base, g_motor_throttle_idle, g_motor_throttle_base_max);

        update_attitude_target();

        const bool apply_pid = (g_motor_throttle_base > (g_motor_throttle_idle + 1.0f));
        if (apply_pid == true)
        {
            // Angle PID: attitude angle error -> target angular rate.
            const float roll_rate_sp = g_pid_roll_angle.update(
                g_target_roll_angle,
                g_attitude_data.roll,
                imu_dt,
                false,
                true
            );

            const float pitch_rate_sp = g_pid_pitch_angle.update(
                g_target_pitch_angle,
                g_attitude_data.pitch,
                imu_dt,
                false,
                true
            );

            // Rate PID: target angular rate - gyro rate -> motor correction.
            float roll_output = g_pid_roll_rate.update(roll_rate_sp, imu_data.gx, imu_dt, false, true);
            float pitch_output = g_pid_pitch_rate.update(pitch_rate_sp, imu_data.gy, imu_dt, false, true);
            float yaw_output = g_pid_yaw_rate.update(g_target_yaw_rate, imu_data.gz, imu_dt, false, true);

            float motor1_speed = g_motor_throttle_base + roll_output - pitch_output + yaw_output; // front left
            float motor2_speed = g_motor_throttle_base - roll_output - pitch_output - yaw_output + 60.0f; // front right
            float motor3_speed = g_motor_throttle_base - roll_output + pitch_output + yaw_output + 120.0f; // rear right
            float motor4_speed = g_motor_throttle_base + roll_output + pitch_output - yaw_output + 10.0f; // rear left

            float max_motor_speed = max(max(motor1_speed, motor2_speed), max(motor3_speed, motor4_speed));
            if (max_motor_speed > g_motor_throttle_max)
            {
                float diff = max_motor_speed - g_motor_throttle_max;
                motor1_speed -= diff;
                motor2_speed -= diff;
                motor3_speed -= diff;
                motor4_speed -= diff;
            }

            float min_motor_speed = min(min(motor1_speed, motor2_speed), min(motor3_speed, motor4_speed));
            if (min_motor_speed < g_motor_throttle_min)
            {
                float diff = g_motor_throttle_min - min_motor_speed;
                motor1_speed += diff;
                motor2_speed += diff;
                motor3_speed += diff;
                motor4_speed += diff;
            }

            g_motors.write_motors(
                motor1_speed,
                motor2_speed,
                motor3_speed,
                motor4_speed
            );
        }
        else
        {
            setup_pid();
            g_motors.write_all_motors(g_motor_throttle_idle);
        }
    }
    else if (g_espnow.is_armed() == false)
    {
        g_was_armed = false;
        g_motor_throttle_base = g_motor_throttle_idle;
        reset_attitude_target();
        setup_pid();
        g_motors.write_all_motors(g_motor_throttle_base_min);
        g_failsafe_active = false;
    }

    if (now - g_espnow_trans_last_time >= g_espnow_trans_period_ms)
    {
        g_espnow_trans_last_time = now;
        g_espnow.send_attitude(g_attitude_data);
    }
    */    
}

static void reset_attitude_target()
{
    g_target_roll_angle = 0.0f;
    g_target_pitch_angle = 0.0f;
}

static void update_attitude_target()
{
    float roll_target = 0.0f;
    float pitch_target = 0.0f;

    if (g_espnow.is_direction_left() == true)
    {
        roll_target = -g_direction_deg;
    }
    else if (g_espnow.is_direction_right() == true)
    {
        roll_target = g_direction_deg;
    }

    if (g_espnow.is_direction_forward() == true)
    {
        pitch_target = g_direction_deg;
    }
    else if (g_espnow.is_direction_backward() == true)
    {
        pitch_target = -g_direction_deg;
    }

    g_target_roll_angle = roll_target;
    g_target_pitch_angle = pitch_target;
}

static float accel_confidence(float ax, float ay, float az)
{
    const float acc_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    const float acc_error = fabsf(acc_norm - 1.0f); // tinh loi acc norm

    if (acc_error <= g_madgwick_acc_error_good) // loi nho hon 0.08f
    {
        return 1.0f;
    }

    if (acc_error >= g_madgwick_acc_error_bad) // loi lon hon 0.25f
    {
        return 0.0f;
    }

    return (g_madgwick_acc_error_bad - acc_error) / (g_madgwick_acc_error_bad - g_madgwick_acc_error_good);
}

static float update_madgwick_beta(float ax, float ay, float az)
{
    const float acc_confidence = accel_confidence(ax, ay, az);

    const bool is_armed = g_espnow.is_armed();
    const bool is_disarmed = ((is_armed == false) && (acc_confidence >= g_madgwick_confidence_min));
    const bool is_flying = ((is_armed == true) && (g_motor_throttle_base >= g_motor_throttle_idle + 100.0f));

    float beta_base = g_madgwick_beta_current;

    if (is_flying == true)
    {
        beta_base = g_madgwick_beta_flying;
    }
    else if (is_armed == true)
    {
        beta_base = g_madgwick_beta_idle;
    }
    else if (is_disarmed == true)
    {
        beta_base = g_madgwick_beta_disarm;
    }

    const float beta_target = g_madgwick_beta_min + (acc_confidence * (beta_base - g_madgwick_beta_min));

    g_madgwick_beta_current += g_madgwick_beta_alpha * (beta_target - g_madgwick_beta_current); // lpf

    return g_madgwick_beta_current;
}

void setup_imu()
{
    const bool b_imu_ready = g_imu.begin(
        &g_spi_bus,
        g_imu_pin_ncs,
        g_imu_pin_sck,
        g_imu_pin_miso,
        g_imu_pin_mosi
    );
    if (b_imu_ready == false)
    {
        Serial.println("MPU6500 NOT DETECTED");
        for (;;) {}
    }
    else
    {
        Serial.println("MPU6500 DETECTED");
    }

    g_imu.set_body_rotation(0.0f, 0.0f, 0.0f);

    /*
     * g_imu.set_body_rotation(0.0f, 0.0f, PI / 2.0f);
     */

    g_imu.calibrate_gyro();

    /*
        Serial.println("\nStart accel 6-side calibration");

        bool g_imu_calibrate_accel_ready = g_imu.calibrate_accel_6_side(1000, &Serial, true);

        if (g_imu_calibrate_accel_ready == true)
        {
            Serial.println("Accel calibration OK");
            g_imu.print_accel_calibration(Serial);
        }
        else
        {
            Serial.println("Accel calibration FAILED");
        }
    */
    g_imu.calibrate_accel_once();

    g_imu.set_gyro_lpf(g_imu_gyro_lpf_alpha);
    g_imu.set_accel_lpf(g_imu_accel_lpf_alpha);

    g_madgwick.begin(g_imu_read_default_hz);
    g_madgwick.setBeta(g_madgwick_beta_current);
}

void setup_pid()
{
    g_pid_roll_angle.reset();
    g_pid_pitch_angle.reset();

    g_pid_roll_rate.reset();
    g_pid_pitch_rate.reset();
    g_pid_yaw_rate.reset();
}

void setup_espnow()
{
    const bool b_espnow_ready = g_espnow.begin();
    if (b_espnow_ready == false)
    {
        Serial.println("ESP-NOW INIT FAILED");
        for (;;) {}
    }
    else
    {
        Serial.println("ESP-NOW INIT SUCCESS");
    }

    const bool b_peer_registered = g_espnow.register_peer();
    if (b_peer_registered == false)
    {
        Serial.println("ESP-NOW PEER REGISTRATION FAILED");
        for (;;) {}
    }
    else
    {
        Serial.println("ESP-NOW PEER REGISTRATION SUCCESS");
    }
}
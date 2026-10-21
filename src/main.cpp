/*
 * main.cpp
 */

#include <Arduino.h>
#include <math.h>
#include "flight_scheduler.h"

#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Exported by Arduino-ESP32 3.3.11 main.cpp. Keep loopTask WDT service.
extern bool loopTaskWDTEnabled;
#if (CONFIG_FREERTOS_UNICORE != 1) || (CONFIG_FREERTOS_HZ != 1000)
#error "Phase 4 continuous loop requires unicore FreeRTOS with a 1000 Hz tick"
#endif

#include "drivers/mpu6500.h"
#include "drivers/adns_3080.h"
#include "drivers/vl53l1x.h"

#include "estimator/madgwick.h"

#include "control/pid_controller.h"

#include "communication/espnow.h"
#include "communication/espnow_protocol.h"
#include "communication/flight_link_safety.h"

#include "hal/motors.h"
#include "hal/i2c_bus.h"
#include "hal/spi_bus.h"

static SPIBus g_spi_bus;
static I2CBus g_i2c_bus;

static MPU6500 g_imu;
static ADNS3080 g_adns;
static VL53L1X g_vl53l1x;
static Madgwick g_madgwick;

static Motors g_motors;

// kp, ki, kd, integLimit, outputLimit, dFilterAlpha
static PIDController g_pid_roll_angle(5.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_pitch_angle(5.0f, 0.1f, 0.01f, 10.0f, 100.0f, 0.5f);

static PIDController g_pid_roll_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_pitch_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);
static PIDController g_pid_yaw_rate(3.0f, 0.01f, 0.005f, 10.0f, 100.0f, 0.5f);

static ESPNow g_espnow;

static attitude_data_packet_t g_attitude_data = {0.0f, 0.0f, 0.0f};

/**
 * @brief Flight target setpoints.
 */
static constexpr float g_target_yaw_rate = 0.0f;
static float g_target_roll_angle = 0.0f;
static float g_target_pitch_angle = 0.0f;

static constexpr float g_direction_deg = 10.0f;

/**
 * @brief Shared SPI bus configuration.
 */
static constexpr uint8_t g_spi_pin_sck = 4u;
static constexpr uint8_t g_spi_pin_miso = 5u;
static constexpr uint8_t g_spi_pin_mosi = 6u;

/**
 * @brief MPU6500 SPI chip-select configuration.
 */
static constexpr uint8_t g_imu_pin_ncs = 7u;

/**
 * @brief ADNS3080 hardware pin configuration.
 */
static constexpr uint8_t g_adns_pin_ncs = 21u;
static constexpr uint8_t g_adns_pin_rst = 20u;
static constexpr int8_t g_adns_pin_npd = -1;

/**
 * @brief Shared I2C bus configuration.
 */
static constexpr uint8_t g_i2c_pin_sda = 8u;
static constexpr uint8_t g_i2c_pin_scl = 9u;
static constexpr uint32_t g_i2c_frequency_hz = 400000u;

/**
 * @brief ADNS3080 sampling configuration.
 *
 * Raw optical-flow sensor data is sampled at 200 Hz.
 */
static constexpr uint32_t g_adns_read_hz = 200u;
static constexpr uint32_t g_adns_read_period_us = 1000000ul / g_adns_read_hz;

static uint32_t g_adns_last_read_time_us = 0u;

/**
 * @brief VL53L1X sampling and polling configuration.
 *
 * Height measurements are targeted at 20 Hz.
 * DATA READY is polled at 1 kHz when a new measurement is expected.
 */
static constexpr uint32_t g_vl53l1x_read_hz = 20u;

static constexpr uint32_t g_vl53l1x_read_period_us = 1000000ul / g_vl53l1x_read_hz;

static constexpr uint32_t g_vl53l1x_poll_period_us = 1000u;

static uint32_t g_vl53l1x_last_read_time_us = 0u;
static uint32_t g_vl53l1x_last_poll_time_us = 0u;

/**
 * @brief VL53L1X measurement configuration.
 */
static constexpr uint32_t g_vl53l1x_timeout_ms = 500u;
static constexpr int16_t g_vl53l1x_offset_mm = 0;

/**
 * @brief MPU6500 sampling configuration.
 */
static constexpr float g_imu_read_default_hz = 1000.0f;

/**
 * @brief MPU6500 low-pass filter configuration.
 */
static constexpr float g_imu_gyro_lpf_alpha = 0.22f;
static constexpr float g_imu_accel_lpf_alpha = 0.10f;

/**
 * @brief Madgwick filter beta configuration for each flight state.
 */
static constexpr float g_madgwick_beta_disarm = 0.10f;
static constexpr float g_madgwick_beta_idle = 0.06f;
static constexpr float g_madgwick_beta_flying = 0.03f;
static constexpr float g_madgwick_beta_min = 0.003f;

/**
 * @brief Madgwick accelerometer confidence thresholds.
 */
static constexpr float g_madgwick_acc_error_good = 0.08f;
static constexpr float g_madgwick_acc_error_bad = 0.25f;

/**
 * @brief Madgwick adaptive beta smoothing configuration.
 */
static constexpr float g_madgwick_beta_alpha = 0.02f;
static constexpr float g_madgwick_confidence_min = 0.90f;

static float g_madgwick_beta_current = g_madgwick_beta_disarm;

/**
 * @brief ESP-NOW telemetry transmission configuration.
 *
 * Telemetry packets are transmitted at 20 Hz.
 */
static constexpr uint32_t g_espnow_trans_period_us = 50000u;

static uint32_t g_espnow_trans_last_time = 0u;
static constexpr uint32_t g_status_trans_period_us = 200000u; // 5 Hz
static uint32_t g_status_trans_last_time = 0u;

/**
 * @brief ESP-NOW heartbeat and communication failsafe configuration.
 */
static constexpr uint32_t g_espnow_heartbeat_timeout_us = 1000000u;
static constexpr uint32_t g_espnow_command_timeout_us = 500000u;
static FlightLinkSafety g_link_safety;
static bool g_flight_armed = false; // Actual ARM state, not requested ARM bit.

/**
 * @brief Base throttle limits.
 *
 * These values define the allowed base throttle range before
 * attitude PID corrections are mixed into individual motors.
 */
static constexpr float g_motor_throttle_base_min = 1000.0f;
static constexpr float g_motor_throttle_base_max = 1850.0f;

/**
 * @brief Motor throttle operating limits.
 */
static constexpr float g_motor_throttle_idle = 1100.0f;
static constexpr float g_motor_throttle_min = 1000.0f;
static constexpr float g_motor_throttle_max = 2000.0f;

/**
 * @brief Manual throttle ramp configuration.
 *
 * Different rates are used when increasing and decreasing throttle.
 */
static constexpr float g_throttle_up_step_per_second = 450.0f;
static constexpr float g_throttle_down_step_per_second = 100.0f;

/**
 * @brief Failsafe throttle ramp-down configuration.
 */
static constexpr float g_failsafe_throttle_step_per_second = 100.0f;

static bool g_failsafe_active = false;

/**
 * @brief Current motor throttle state.
 */
static float g_motor_throttle_base = g_motor_throttle_idle;

static bool g_was_armed = false;

/**
 * @brief MPU6500 data watchdog.
 *
 * Radio failsafe keeps its existing ramp to idle (1100). An IMU fault is
 * different: without attitude feedback, motors are cut to OFF (1000).
 * The fault stays latched until valid IMU samples and a fresh DISARM arrive.
 */
static constexpr uint32_t g_imu_data_timeout_us = 150000u;
static uint32_t g_imu_last_valid_us = 0u;
static bool g_imu_seen_valid = false;
static bool g_imu_fault_latched = false;

static void setup_spi_bus();
static void setup_i2c_bus();
static void setup_imu();
static void setup_vl53l1x();
static void setup_pid();
static void setup_espnow();

static float accel_confidence(float ax, float ay, float az);
static float update_madgwick_beta(float ax, float ay, float az);

static void update_attitude_target(const command_data_packet_t &cmd);
static void reset_attitude_target();
static void send_flight_status(
    const ESPNowRxSnapshot &rx, uint32_t rx_check_now, bool cmd_recent, bool hb_recent, FlightLinkReason reason);

void setup()
{
    setup_spi_bus();
    setup_i2c_bus();
    setup_imu();
    // ADNS3080 is intentionally not initialized in this firmware variant.
    setup_vl53l1x();
    setup_pid();
    setup_espnow();

    g_motors.setup_motors();
    g_imu_last_valid_us = micros();
}

static void flight_control_pass()
{
    const uint32_t now = micros();

    // Check the remote link on EVERY pass, including passes without a new
    // IMU sample. IMU sampling must never block DISARM, timeout or status.
    const ESPNowRxSnapshot reset_rx = g_espnow.get_rx_snapshot();
    const uint32_t reset_check_now = micros();
    if ((reset_rx.command.reset == true) &&
        (ESPNow::is_fresh(
             reset_check_now, reset_rx.last_command_us, reset_rx.have_command, g_espnow_command_timeout_us) == true))
    {
        g_motor_throttle_base = g_motor_throttle_idle;
        g_motors.write_all_motors(g_motor_throttle_base_min);
        g_espnow.reset_command();
        delay(100);
        esp_restart();
        return;
    }

    const ESPNowRxSnapshot rx = g_espnow.get_rx_snapshot();
    const uint32_t rx_check_now = micros();
    const bool hb_recent =
        (ESPNow::is_fresh(rx_check_now, rx.last_heartbeat_us, rx.have_heartbeat, g_espnow_heartbeat_timeout_us) ==
         true);
    const bool cmd_recent =
        (ESPNow::is_fresh(rx_check_now, rx.last_command_us, rx.have_command, g_espnow_command_timeout_us) == true);

    const bool sticks_neutral = ((rx.command.throttle_up == false) && (rx.command.throttle_down == false) &&
                                 (rx.command.direction_forward == false) && (rx.command.direction_backward == false) &&
                                 (rx.command.direction_left == false) && (rx.command.direction_right == false));
    const FlightLinkResult link = g_link_safety.update({cmd_recent, hb_recent, rx.command.arm, sticks_neutral});

    // A false return normally means "no NEW IMU sample yet", not a fault.
    // Only a prolonged absence of valid samples, or invalid IMU data, faults.
    const bool imu_updated = (g_imu.update() == true);
    bool imu_sample_valid = false;
    if (imu_updated == true)
    {
        const auto &sample = g_imu.get_filtered();
        const float sample_dt = g_imu.get_timing().dt;
        imu_sample_valid =
            ((static_cast<bool>(isfinite(sample.ax)) == true) && (static_cast<bool>(isfinite(sample.ay)) == true) &&
             (static_cast<bool>(isfinite(sample.az)) == true) && (static_cast<bool>(isfinite(sample.gx)) == true) &&
             (static_cast<bool>(isfinite(sample.gy)) == true) && (static_cast<bool>(isfinite(sample.gz)) == true) &&
             (static_cast<bool>(isfinite(sample_dt)) == true) && (sample_dt > 0.0f) && (sample_dt < 0.1f));
        if (imu_sample_valid == true)
        {
            g_imu_last_valid_us = micros();
            g_imu_seen_valid = true;
        }
    }

    const bool imu_recent =
        ((g_imu_seen_valid == true) && (uint32_t(micros() - g_imu_last_valid_us) <= g_imu_data_timeout_us));
    if (((imu_updated == true) && (imu_sample_valid == false)) ||
        (uint32_t(micros() - g_imu_last_valid_us) > g_imu_data_timeout_us))
    {
        g_imu_fault_latched = true;
    }

    // A valid sample and a NEW DISARM command are required to clear the IMU
    // fault. FlightLinkSafety independently requires a DISARM -> ARM edge.
    if ((g_imu_fault_latched == true) && (imu_sample_valid == true) && (imu_recent == true) && (cmd_recent == true) &&
        (rx.command.arm == false))
    {
        g_imu_fault_latched = false;
    }

    g_flight_armed = ((link.armed == true) && (imu_recent == true) && (g_imu_fault_latched == false));
    g_failsafe_active = ((link.failsafe == true) || (g_imu_fault_latched == true));
    if (link.entered_failsafe == true)
    {
        reset_attitude_target();
    }

    // DISARM must cut outputs even when the IMU has no new sample.
    if (g_flight_armed == false)
    {
        g_motors.write_all_motors(g_motor_throttle_min);
    }

    const FlightLinkReason status_reason = (g_imu_fault_latched == true) ? FlightLinkReason::IMU_TIMEOUT : link.reason;

    if (g_imu_fault_latched == true)
    {
        // No attitude feedback: do NOT continue the normal radio failsafe
        // ramp or reuse stale PID outputs. Immediately command all motors OFF.
        g_was_armed = false;
        g_motor_throttle_base = g_motor_throttle_idle;
        reset_attitude_target();
        setup_pid();
        g_motors.write_all_motors(g_motor_throttle_min);
        send_flight_status(rx, rx_check_now, cmd_recent, hb_recent, status_reason);
        return;
    }

    if (imu_sample_valid == false)
    {
        send_flight_status(rx, rx_check_now, cmd_recent, hb_recent, status_reason);
        return;
    }

    const auto &imu_data = g_imu.get_filtered();
    const float imu_dt = g_imu.get_timing().dt;

    const float beta = update_madgwick_beta(imu_data.ax, imu_data.ay, imu_data.az);
    g_madgwick.setBeta(beta);
    g_madgwick.updateIMUdt(imu_data.gx, imu_data.gy, imu_data.gz, imu_data.ax, imu_data.ay, imu_data.az, imu_dt);
    g_attitude_data.roll = g_madgwick.getRoll();
    g_attitude_data.pitch = g_madgwick.getPitch();
    g_attitude_data.yaw = g_madgwick.getYaw();

    if ((isfinite(g_attitude_data.roll) == false) || (isfinite(g_attitude_data.pitch) == false) ||
        (isfinite(g_attitude_data.yaw) == false))
    {
        g_imu_fault_latched = true;
        g_flight_armed = false;
        g_failsafe_active = true;
        g_was_armed = false;
        g_motor_throttle_base = g_motor_throttle_idle;
        reset_attitude_target();
        setup_pid();
        g_motors.write_all_motors(g_motor_throttle_min);

        // Restart only a corrupted filter state; Madgwick math and tuning stay unchanged.
        g_madgwick = Madgwick();
        g_madgwick.begin(g_imu_read_default_hz);
        g_madgwick.setBeta(g_madgwick_beta_current);
        send_flight_status(rx, rx_check_now, cmd_recent, hb_recent, FlightLinkReason::IMU_TIMEOUT);
        return;
    }

    if ((g_flight_armed == true) && (g_failsafe_active == true))
    {
        g_motor_throttle_base -= g_failsafe_throttle_step_per_second * imu_dt;
        g_motor_throttle_base = max(g_motor_throttle_base, g_motor_throttle_idle);

        const bool apply_pid = (g_motor_throttle_base > (g_motor_throttle_idle + 1.0f));
        if (apply_pid == true)
        {
            // Angle PID: attitude angle error -> target angular rate.
            const float roll_rate_sp =
                g_pid_roll_angle.update(g_target_roll_angle, g_attitude_data.roll, imu_dt, false, true);

            const float pitch_rate_sp =
                g_pid_pitch_angle.update(g_target_pitch_angle, g_attitude_data.pitch, imu_dt, false, true);

            // Rate PID: target angular rate - gyro rate -> motor correction.
            float roll_output = g_pid_roll_rate.update(roll_rate_sp, imu_data.gx, imu_dt, false, true);
            float pitch_output = g_pid_pitch_rate.update(pitch_rate_sp, imu_data.gy, imu_dt, false, true);
            float yaw_output = g_pid_yaw_rate.update(g_target_yaw_rate, imu_data.gz, imu_dt, false, true);

            float motor1_speed = g_motor_throttle_base + roll_output - pitch_output + yaw_output; // front left
            float motor2_speed = g_motor_throttle_base - roll_output - pitch_output - yaw_output; // front right
            float motor3_speed = g_motor_throttle_base - roll_output + pitch_output + yaw_output; // rear right
            float motor4_speed = g_motor_throttle_base + roll_output + pitch_output - yaw_output + 100.0f; // rear left

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
            g_motors.write_motors(motor1_speed, motor2_speed, motor3_speed, motor4_speed);
        }
        else
        {
            setup_pid();
            g_motors.write_all_motors(g_motor_throttle_idle);
        }
    }
    else if ((g_flight_armed == true) && (g_failsafe_active == false))
    {
        if (g_was_armed == false)
        {
            g_motor_throttle_base = g_motor_throttle_idle;
            g_was_armed = true;
        }

        if (rx.command.throttle_up == true)
        {
            g_motor_throttle_base += g_throttle_up_step_per_second * imu_dt;
        }
        else if (rx.command.throttle_down == true)
        {
            g_motor_throttle_base -= g_throttle_down_step_per_second * imu_dt;
        }

        g_motor_throttle_base = constrain(g_motor_throttle_base, g_motor_throttle_idle, g_motor_throttle_base_max);

        update_attitude_target(rx.command);

        const bool apply_pid = (g_motor_throttle_base > (g_motor_throttle_idle + 1.0f));
        if (apply_pid == true)
        {
            // Angle PID: attitude angle error -> target angular rate.
            const float roll_rate_sp =
                g_pid_roll_angle.update(g_target_roll_angle, g_attitude_data.roll, imu_dt, false, true);

            const float pitch_rate_sp =
                g_pid_pitch_angle.update(g_target_pitch_angle, g_attitude_data.pitch, imu_dt, false, true);

            // Rate PID: target angular rate - gyro rate -> motor correction.
            float roll_output = g_pid_roll_rate.update(roll_rate_sp, imu_data.gx, imu_dt, false, true);
            float pitch_output = g_pid_pitch_rate.update(pitch_rate_sp, imu_data.gy, imu_dt, false, true);
            float yaw_output = g_pid_yaw_rate.update(g_target_yaw_rate, imu_data.gz, imu_dt, false, true);

            float motor1_speed = g_motor_throttle_base + roll_output - pitch_output + yaw_output; // front left
            float motor2_speed = g_motor_throttle_base - roll_output - pitch_output - yaw_output; // front right
            float motor3_speed = g_motor_throttle_base - roll_output + pitch_output + yaw_output; // rear right
            float motor4_speed = g_motor_throttle_base + roll_output + pitch_output - yaw_output + 100.0f; // rear left

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
            g_motors.write_motors(motor1_speed, motor2_speed, motor3_speed, motor4_speed);
        }
        else
        {
            setup_pid();
            g_motors.write_all_motors(g_motor_throttle_idle);
        }
    }
    else if (g_flight_armed == false)
    {
        g_was_armed = false;
        g_motor_throttle_base = g_motor_throttle_idle;
        reset_attitude_target();
        setup_pid();
        g_motors.write_all_motors(g_motor_throttle_base_min);

        // Failsafe latch is cleared ONLY by a fresh DISARM packet in FlightLinkSafety.
    }

    if (now - g_espnow_trans_last_time >= g_espnow_trans_period_us)
    {
        g_espnow_trans_last_time = now;
        g_espnow.send_attitude(g_attitude_data); // Legacy 12-byte attitude packet unchanged.
    }

    send_flight_status(rx, rx_check_now, cmd_recent, hb_recent, status_reason);
}

void loop()
{
    // Keep the control pass in Arduino's original loopTask (priority 1).
    // The regular 1-tick release avoids Arduino's 5-tick delay every ~2 s.
    uint32_t last_release_us = micros();
    for (;;)
    {
        if (loopTaskWDTEnabled)
        {
            esp_task_wdt_reset();
        }
        flight_control_pass();
        const uint32_t release_now_us = micros();
        if (flight_scheduler::release_due(release_now_us, last_release_us))
        {
            vTaskDelay(flight_scheduler::kReleaseTicks);
        }
    }
}

static void send_flight_status(
    const ESPNowRxSnapshot &rx, uint32_t rx_check_now, bool cmd_recent, bool hb_recent, FlightLinkReason reason)
{
    const uint32_t now = micros();

    // Keep the original 5 Hz status and stagger it from attitude telemetry.
    if ((uint32_t(now - g_status_trans_last_time) < g_status_trans_period_us) ||
        (uint32_t(now - g_espnow_trans_last_time) < 12000u))
    {
        return;
    }

    g_status_trans_last_time = now;
    const bool imu_ready = ((g_imu_fault_latched == false) && (g_imu_seen_valid == true) &&
                            (uint32_t(now - g_imu_last_valid_us) <= g_imu_data_timeout_us));
    const flight_status_packet_t status = {
        FLIGHT_STATUS_MAGIC,
        millis(),
        (rx.have_command == true) ? (uint32_t(rx_check_now - rx.last_command_us) / 1000u) : UINT32_MAX,
        uint16_t((g_flight_armed == true) ? g_motor_throttle_base : g_motor_throttle_min),
        uint8_t(((g_flight_armed == true) ? FLIGHT_STATUS_ARMED : 0u) |
                ((g_failsafe_active == true) ? FLIGHT_STATUS_FAILSAFE : 0u) |
                ((hb_recent == true) ? FLIGHT_STATUS_HB_RECENT : 0u) |
                ((cmd_recent == true) ? FLIGHT_STATUS_CMD_RECENT : 0u) |
                ((imu_ready == true) ? FLIGHT_STATUS_IMU_READY : 0u)),
        uint8_t(reason)};
    g_espnow.send_status(status);
}

static void reset_attitude_target()
{
    g_target_roll_angle = 0.0f;
    g_target_pitch_angle = 0.0f;
}

static void update_attitude_target(const command_data_packet_t &cmd)
{
    float roll_target = 0.0f;
    float pitch_target = 0.0f;

    if (cmd.direction_left == true)
    {
        roll_target = -g_direction_deg;
    }
    else if (cmd.direction_right == true)
    {
        roll_target = g_direction_deg;
    }

    if (cmd.direction_forward == true)
    {
        pitch_target = g_direction_deg;
    }
    else if (cmd.direction_backward == true)
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

    const bool is_armed = (g_flight_armed == true);
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

void setup_spi_bus()
{
    /*
     * Deselect every device sharing the physical SPI bus before SPI starts.
     */
    pinMode(g_imu_pin_ncs, OUTPUT);
    digitalWrite(g_imu_pin_ncs, HIGH);

    pinMode(g_adns_pin_ncs, OUTPUT);
    digitalWrite(g_adns_pin_ncs, HIGH);

    const bool b_spi_ready = (g_spi_bus.begin(g_spi_pin_sck, g_spi_pin_miso, g_spi_pin_mosi) == true);

    if (b_spi_ready == false)
    {
        for (;;)
        {
        }
    }
}

void setup_i2c_bus()
{
    const bool b_i2c_ready = (g_i2c_bus.begin(g_i2c_pin_sda, g_i2c_pin_scl, g_i2c_frequency_hz) == true);

    if (b_i2c_ready == false)
    {
        for (;;)
        {
        }
    }
}

void setup_imu()
{
    const bool b_imu_ready = (g_imu.begin(&g_spi_bus, g_imu_pin_ncs) == true);
    if (b_imu_ready == false)
    {
        for (;;)
        {
        }
    }

    g_imu.set_body_rotation(0.0f, 0.0f, 0.0f);

    /*
     * g_imu.set_body_rotation(0.0f, 0.0f, PI / 2.0f);
     */

    g_imu.calibrate_gyro();

    g_imu.calibrate_accel_once();

    g_imu.set_gyro_lpf(g_imu_gyro_lpf_alpha);
    g_imu.set_accel_lpf(g_imu_accel_lpf_alpha);

    g_madgwick.begin(g_imu_read_default_hz);
    g_madgwick.setBeta(g_madgwick_beta_current);
}

void setup_vl53l1x()
{
    const bool b_vl53l1x_ready =
        (g_vl53l1x.begin(&g_i2c_bus, VL53L1X::default_i2c_address, g_vl53l1x_timeout_ms, g_vl53l1x_offset_mm) == true);

    if (b_vl53l1x_ready == false)
    {
        for (;;)
        {
        }
    }
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
    const bool b_espnow_ready = (g_espnow.begin() == true);
    if (b_espnow_ready == false)
    {
        for (;;)
        {
        }
    }

    const bool b_peer_registered = (g_espnow.register_peer() == true);
    if (b_peer_registered == false)
    {
        for (;;)
        {
        }
    }
}

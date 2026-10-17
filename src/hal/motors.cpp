/*
 * motor.cpp
 * Motor PWM control implementation.
 */

#include "hal/motors.h"

uint32_t Motors::throttle_to_pwm(float throttle)
{
    const float clamped = constrain(throttle, motor_speed_min_us, motor_speed_max_us);
    const float pwm = (clamped - motor_speed_min_us) * motor_pwm_max / (motor_speed_max_us - motor_speed_min_us);

    return static_cast<uint32_t>(pwm);
}

void Motors::write_motors(float motor_1_throttle,
                          float motor_2_throttle,
                          float motor_3_throttle,
                          float motor_4_throttle)
{
    ledcWrite(motor_1_pin, throttle_to_pwm(motor_1_throttle));
    ledcWrite(motor_2_pin, throttle_to_pwm(motor_2_throttle));
    ledcWrite(motor_3_pin, throttle_to_pwm(motor_3_throttle));
    ledcWrite(motor_4_pin, throttle_to_pwm(motor_4_throttle));
}

void Motors ::write_all_motors(float throttle)
{
    write_motors(throttle, throttle, throttle, throttle);
}

void Motors::setup_motors()
{
    ledcAttach(motor_1_pin, motor_freq_hz, motor_resolution_bits);
    ledcAttach(motor_2_pin, motor_freq_hz, motor_resolution_bits);
    ledcAttach(motor_3_pin, motor_freq_hz, motor_resolution_bits);
    ledcAttach(motor_4_pin, motor_freq_hz, motor_resolution_bits);

    write_all_motors(motor_speed_min_us);
}

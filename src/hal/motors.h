/*
 * motors.h
 * Motor PWM control interface.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors
{
public:
    void setup_motors();
    void write_motors(float motor_1_us, float motor_2_us, float motor_3_us, float motor_4_us);
    void write_all_motors(float throttle);

private:
    static uint32_t throttle_to_pwm(float throttle);

    static constexpr uint8_t motor_1_pin = 10u; // Front-left motor.
    static constexpr uint8_t motor_2_pin = 1u;  // Front-right motor.
    static constexpr uint8_t motor_3_pin = 3u;  // Back-right motor.
    static constexpr uint8_t motor_4_pin = 0u;  // Back-left motor.

    static constexpr uint32_t motor_freq_hz = 24000u; // 24 kHz.
    static constexpr uint8_t motor_resolution_bits = 10u;

    static constexpr float motor_pwm_max = 1023.0f;

    static constexpr float motor_speed_min_us = 1000.0f;
    static constexpr float motor_speed_max_us = 2000.0f;

    static constexpr uint8_t motor_channel_1 = 0u;
    static constexpr uint8_t motor_channel_2 = 1u;
    static constexpr uint8_t motor_channel_3 = 2u;
    static constexpr uint8_t motor_channel_4 = 3u;
};

#endif /* MOTORS_H */

/*
 * mpu6500.cpp
 * MPU6500 IMU driver implementation.
 */

#include "drivers/mpu6500.h"
#include <math.h>
#include <float.h>

float MPU6500::accel_raw_to_g(int16_t raw_value)
{
    return static_cast<float>(raw_value) / accel_divisor;
}

float MPU6500::gyro_raw_to_deg_s(int16_t raw_value)
{
    return static_cast<float>(raw_value) / gyro_divisor;
}

float MPU6500::clamp_alpha(float alpha)
{
    if (alpha < 0.0f)
    {
        return 0.0f;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

float MPU6500::lpf_step(float current, float target, float alpha)
{
    return current + (alpha * (target - current));
}


float MPU6500::safe_divisor(float value)
{
    if (fabsf(value) < 1e-6f)
    {
        return 1.0f;
    }

    return value;
}

float MPU6500::safe_accel_scale(float max_value, float min_value)
{
    const float scale = (max_value - min_value) / (2.0f * one_g);
    return safe_divisor(scale);
}

void MPU6500::wait_for_stream_input(Stream * p_stream)
{
    if (p_stream == nullptr)
    {
        delay(2000);
        return;
    }

    while (p_stream->available() > 0)
    {
        (void)p_stream->read();
    }

    while (p_stream->available() == 0)
    {
        delay(10);
    }

    while (p_stream->available() > 0)
    {
        (void)p_stream->read();
    }

    delay(300);
}

bool MPU6500::read_accel_average_g(uint16_t samples, float & avg_x, float & avg_y, float & avg_z)
{
    if ((samples == 0u) || (p_spi_bus == nullptr))
    {
        return false;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    for (uint16_t sample_index = 0u; sample_index < samples; ++sample_index)
    {
        if (wait_for_data(10000u) == false)
        {
            return false;
        }

        if (read_sensor() == false)
        {
            return false;
        }

        sum_x += static_cast<double>(accel_raw_to_g(raw.ax));
        sum_y += static_cast<double>(accel_raw_to_g(raw.ay));
        sum_z += static_cast<double>(accel_raw_to_g(raw.az));
    }

    avg_x = static_cast<float>(sum_x / static_cast<double>(samples));
    avg_y = static_cast<float>(sum_y / static_cast<double>(samples));
    avg_z = static_cast<float>(sum_z / static_cast<double>(samples));

    return true;
}

bool MPU6500::begin(SPIBus * p_spi_bus_ref, uint8_t chip_select_pin_ref)
{
    if (p_spi_bus_ref == nullptr)
    {
        return false;
    }

    p_spi_bus = p_spi_bus_ref;
    chip_select_pin = chip_select_pin_ref;

    pinMode(chip_select_pin, OUTPUT);
    digitalWrite(chip_select_pin, HIGH);

    if (p_spi_bus->is_initialized() == false)
    {
        return false;
    }

    delay(100);

    if (validate_whoami() == false)
    {
        return false;
    }

    /*
     * Wake up MPU6500.
     */
    p_spi_bus->write_register(chip_select_pin, pwr_mgmt_1_reg, 0x00u, spi_settings);
    delay(100);

    /*
     * CONFIG      = 0x01: DLPF config
     * ACCEL_CFG2  = 0x01: accel DLPF config
     * SMPLRT_DIV  = 0x00: maximum sample rate from internal sampling
     * GYRO_CONFIG = 0x18: ±2000 dps
     * ACCEL_CONFIG= 0x08: ±4g
     */
    p_spi_bus->write_register(chip_select_pin, config_reg, 0x01u, spi_settings);
    p_spi_bus->write_register(chip_select_pin, accel_config_2_reg, 0x01u, spi_settings);
    p_spi_bus->write_register(chip_select_pin, smplrt_div_reg, 0x00u, spi_settings);

    p_spi_bus->write_register(chip_select_pin, gyro_config_reg, 0x18u, spi_settings);
    p_spi_bus->write_register(chip_select_pin, accel_config_reg, 0x08u, spi_settings);

    /*
     * Enable data-ready status bit.
     */
    p_spi_bus->write_register(chip_select_pin, int_enable_reg, 0x01u, spi_settings);

    /*
     * Clear any stale interrupt status.
     */
    p_spi_bus->read_register(chip_select_pin, int_status_reg, spi_settings);

    timing.last_us = micros();
    timing.now_us = timing.last_us;
    timing.dt = 0.0f;
    timing.imu_dt = 0.0f;

    reset_filters();

    return true;
}

bool MPU6500::validate_whoami()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    const uint8_t whoami = p_spi_bus->read_register(chip_select_pin, who_am_i_reg, spi_settings);

    return ((whoami == 0x70u) || (whoami == 0x71u));
}

bool MPU6500::data_ready()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    const uint8_t status = p_spi_bus->read_register(chip_select_pin, int_status_reg, spi_settings);
    return ((status & data_ready_mask) != 0u);
}

bool MPU6500::wait_for_data(uint32_t timeout_us)
{
    const uint32_t start_us = micros();

    while (data_ready() == false)
    {
        if ((micros() - start_us) >= timeout_us)
        {
            return false;
        }
    }

    return true;
}

bool MPU6500::update()
{
    if (wait_for_data() == false)
    {
        return false;
    }

    timing.now_us = micros();
    timing.dt = static_cast<float>(timing.now_us - timing.last_us) * 1e-6f;
    timing.imu_dt = timing.dt;
    timing.last_us = timing.now_us;

    if (read_sensor() == false)
    {
        return false;
    }

    scale_sensor();
    apply_bias_and_scale();
    rotate_to_body_frame();
    low_pass_filter();

    return true;
}

bool MPU6500::read_sensor()
{
    if (p_spi_bus == nullptr)
    {
        return false;
    }

    uint8_t buffer[14];

    if (p_spi_bus->read_registers(chip_select_pin, accel_xout_h_reg, buffer, 14, spi_settings) == false)
    {
        return false;
    }

    raw.ax = static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);
    raw.ay = static_cast<int16_t>((static_cast<uint16_t>(buffer[2]) << 8) | buffer[3]);
    raw.az = static_cast<int16_t>((static_cast<uint16_t>(buffer[4]) << 8) | buffer[5]);

    /*
     * buffer[6] and buffer[7] are temperature. Ignored here.
     */
    raw.gx = static_cast<int16_t>((static_cast<uint16_t>(buffer[8]) << 8) | buffer[9]);
    raw.gy = static_cast<int16_t>((static_cast<uint16_t>(buffer[10]) << 8) | buffer[11]);
    raw.gz = static_cast<int16_t>((static_cast<uint16_t>(buffer[12]) << 8) | buffer[13]);

    return true;
}

void MPU6500::scale_sensor()
{
    /*
     * Madgwick version you use expects:
     * gyro  = deg/s
     * accel = any consistent unit, here g
     */
    scaled.ax = accel_raw_to_g(raw.ax);
    scaled.ay = accel_raw_to_g(raw.ay);
    scaled.az = accel_raw_to_g(raw.az);

    scaled.gx = gyro_raw_to_deg_s(raw.gx);
    scaled.gy = gyro_raw_to_deg_s(raw.gy);
    scaled.gz = gyro_raw_to_deg_s(raw.gz);
}

void MPU6500::apply_bias_and_scale()
{
    /*
     * Gyro:
     * deg/s - deg/s bias
     */
    scaled.gx -= gyro_bias_x;
    scaled.gy -= gyro_bias_y;
    scaled.gz -= gyro_bias_z;

    /*
     * Accel:
     * Flix-style correction: (acc - bias) / scale.
     * For now scale defaults to 1.0 unless you add 6-side calibration later.
     */
    scaled.ax = (scaled.ax - accel_bias_x) / accel_scale_x;
    scaled.ay = (scaled.ay - accel_bias_y) / accel_scale_y;
    scaled.az = (scaled.az - accel_bias_z) / accel_scale_z;
}

void MPU6500::set_body_rotation(float roll_rad, float pitch_rad, float yaw_rad)
{
    imu_rotation_roll = roll_rad;
    imu_rotation_pitch = pitch_rad;
    imu_rotation_yaw = yaw_rad;

    reset_filters();
}

void MPU6500::rotate_vector_to_body(float & x, float & y, float & z) const
{
    const float cr = cosf(imu_rotation_roll);
    const float sr = sinf(imu_rotation_roll);

    const float cp = cosf(imu_rotation_pitch);
    const float sp = sinf(imu_rotation_pitch);

    const float cy = cosf(imu_rotation_yaw);
    const float sy = sinf(imu_rotation_yaw);

    /*
     * R = Rz(yaw) * Ry(pitch) * Rx(roll)
     *
     * The stored rotation describes the IMU mounting rotation.
     * To transform sensor-frame vector into body-frame vector,
     * use inverse(R). Since R is orthonormal, inverse(R) = transpose(R).
     */
    const float r00 = cy * cp;
    const float r01 = (cy * sp * sr) - (sy * cr);
    const float r02 = (cy * sp * cr) + (sy * sr);

    const float r10 = sy * cp;
    const float r11 = (sy * sp * sr) + (cy * cr);
    const float r12 = (sy * sp * cr) - (cy * sr);

    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    const float old_x = x;
    const float old_y = y;
    const float old_z = z;

    /*
     * body = transpose(R) * sensor
     */
    x = (r00 * old_x) + (r10 * old_y) + (r20 * old_z);
    y = (r01 * old_x) + (r11 * old_y) + (r21 * old_z);
    z = (r02 * old_x) + (r12 * old_y) + (r22 * old_z);
}

void MPU6500::rotate_to_body_frame()
{
    rotate_vector_to_body(scaled.ax, scaled.ay, scaled.az);
    rotate_vector_to_body(scaled.gx, scaled.gy, scaled.gz);
}

void MPU6500::low_pass_filter()
{
    if (filters_initialized == false)
    {
        filtered = scaled;
        filters_initialized = true;
        return;
    }

    filtered.gx = lpf_step(filtered.gx, scaled.gx, gyro_alpha);
    filtered.gy = lpf_step(filtered.gy, scaled.gy, gyro_alpha);
    filtered.gz = lpf_step(filtered.gz, scaled.gz, gyro_alpha);

    filtered.ax = lpf_step(filtered.ax, scaled.ax, accel_alpha);
    filtered.ay = lpf_step(filtered.ay, scaled.ay, accel_alpha);
    filtered.az = lpf_step(filtered.az, scaled.az, accel_alpha);
}

void MPU6500::calibrate_gyro(uint16_t samples)
{
    if (samples == 0u)
    {
        return;
    }

    int64_t sum_raw_x = 0;
    int64_t sum_raw_y = 0;
    int64_t sum_raw_z = 0;

    for (uint16_t sample_index = 0u; sample_index < samples; ++sample_index)
    {
        read_sensor();

        sum_raw_x += raw.gx;
        sum_raw_y += raw.gy;
        sum_raw_z += raw.gz;

        delay(2);
    }

    gyro_bias_x = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_x / samples));
    gyro_bias_y = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_y / samples));
    gyro_bias_z = gyro_raw_to_deg_s(static_cast<int16_t>(sum_raw_z / samples));

    reset_filters();
}

void MPU6500::calibrate_accel_once(uint16_t samples)
{
    if (samples == 0u)
    {
        return;
    }

    int64_t sum_raw_x = 0;
    int64_t sum_raw_y = 0;
    int64_t sum_raw_z = 0;

    for (uint16_t sample_index = 0u; sample_index < samples; ++sample_index)
    {
        read_sensor();

        sum_raw_x += raw.ax;
        sum_raw_y += raw.ay;
        sum_raw_z += raw.az;

        delay(2);
    }

    const float avg_x = accel_raw_to_g(static_cast<int16_t>(sum_raw_x / samples));
    const float avg_y = accel_raw_to_g(static_cast<int16_t>(sum_raw_y / samples));
    const float avg_z = accel_raw_to_g(static_cast<int16_t>(sum_raw_z / samples));

    /*
     * One-position accel calibration.
     *
     * This assumes the board is level during calibration and sensor Z reads +1g.
     * If your sensor reads -1g when level, change:
     *
     * accel_bias_z = avg_z - one_g;
     *
     * to:
     *
     * accel_bias_z = avg_z + one_g;
     */
    accel_bias_x = avg_x;
    accel_bias_y = avg_y;
    accel_bias_z = avg_z - one_g;

    /*
     * Scale remains 1.0 until you implement 6-side accel calibration.
     */
    accel_scale_x = 1.0f;
    accel_scale_y = 1.0f;
    accel_scale_z = 1.0f;

    reset_filters();
}

bool MPU6500::calibrate_accel_6_side(uint16_t samples_per_side, Stream * p_stream, bool wait_for_user)
{
    if ((samples_per_side == 0u) || (p_spi_bus == nullptr))
    {
        return false;
    }

    const char * side_names[6] =
    {
        "+Z up / level",
        "-Z up / upside down",
        "+X up / right or left side",
        "-X up / opposite side",
        "+Y up / nose or tail up",
        "-Y up / opposite nose or tail"
    };

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;

    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    float max_z = -FLT_MAX;

    /*
     * Use raw accel converted to g, so the existing calibration values do not
     * affect this calibration run.
     */
    for (uint8_t side = 0u; side < 6u; ++side)
    {
        if (p_stream != nullptr)
        {
            p_stream->println();
            p_stream->print(F("Accel 6-side calibration: place IMU/drone on side "));
            p_stream->print(side + 1u);
            p_stream->print(F("/6: "));
            p_stream->println(side_names[side]);
            p_stream->println(F("Keep it still. Send any key/newline to sample."));
        }

        if (wait_for_user)
        {
            wait_for_stream_input(p_stream);
        }
        else
        {
            delay(1200);
        }

        float avg_x = 0.0f;
        float avg_y = 0.0f;
        float avg_z = 0.0f;

        if (read_accel_average_g(samples_per_side, avg_x, avg_y, avg_z) == false)
        {
            if (p_stream != nullptr)
            {
                p_stream->println(F("Accel calibration failed while sampling."));
            }

            return false;
        }

        if (avg_x < min_x) { min_x = avg_x; }
        if (avg_y < min_y) { min_y = avg_y; }
        if (avg_z < min_z) { min_z = avg_z; }

        if (avg_x > max_x) { max_x = avg_x; }
        if (avg_y > max_y) { max_y = avg_y; }
        if (avg_z > max_z) { max_z = avg_z; }

        if (p_stream != nullptr)
        {
            p_stream->print(F("Average accel [g]: x="));
            p_stream->print(avg_x, 6);
            p_stream->print(F(" y="));
            p_stream->print(avg_y, 6);
            p_stream->print(F(" z="));
            p_stream->println(avg_z, 6);
        }
    }

    const float span_x = max_x - min_x;
    const float span_y = max_y - min_y;
    const float span_z = max_z - min_z;

    /*
     * A valid 6-side run should produce about 2g span on every axis.
     * Keep the threshold loose so small placement errors do not fail the run,
     * but still catch missing/repeated faces.
     */
    if ((span_x < 1.2f) || (span_y < 1.2f) || (span_z < 1.2f))
    {
        if (p_stream != nullptr)
        {
            p_stream->println(F("Accel calibration failed: each axis must see both +1g and -1g."));
            p_stream->print(F("Spans [g]: x="));
            p_stream->print(span_x, 6);
            p_stream->print(F(" y="));
            p_stream->print(span_y, 6);
            p_stream->print(F(" z="));
            p_stream->println(span_z, 6);
        }

        return false;
    }

    accel_bias_x = (max_x + min_x) * 0.5f;
    accel_bias_y = (max_y + min_y) * 0.5f;
    accel_bias_z = (max_z + min_z) * 0.5f;

    accel_scale_x = safe_accel_scale(max_x, min_x);
    accel_scale_y = safe_accel_scale(max_y, min_y);
    accel_scale_z = safe_accel_scale(max_z, min_z);

    /*
     * Avoid mixing pre-calibration filtered output with post-calibration data.
     */
    reset_filters();

    if (p_stream != nullptr)
    {
        p_stream->println(F("Accel 6-side calibration complete."));
        print_accel_calibration(*p_stream);
    }

    return true;
}

void MPU6500::set_accel_calibration(float bias_x, float bias_y, float bias_z,
                                    float scale_x, float scale_y, float scale_z)
{
    accel_bias_x = bias_x;
    accel_bias_y = bias_y;
    accel_bias_z = bias_z;

    accel_scale_x = safe_divisor(scale_x);
    accel_scale_y = safe_divisor(scale_y);
    accel_scale_z = safe_divisor(scale_z);

    reset_filters();
}

MPU6500::accel_calibration_t MPU6500::get_accel_calibration() const
{
    accel_calibration_t calibration;

    calibration.bias_x = accel_bias_x;
    calibration.bias_y = accel_bias_y;
    calibration.bias_z = accel_bias_z;

    calibration.scale_x = accel_scale_x;
    calibration.scale_y = accel_scale_y;
    calibration.scale_z = accel_scale_z;

    return calibration;
}

void MPU6500::print_accel_calibration(Stream & stream) const
{
    stream.print(F("accel_bias_x="));
    stream.println(accel_bias_x, 8);
    stream.print(F("accel_bias_y="));
    stream.println(accel_bias_y, 8);
    stream.print(F("accel_bias_z="));
    stream.println(accel_bias_z, 8);

    stream.print(F("accel_scale_x="));
    stream.println(accel_scale_x, 8);
    stream.print(F("accel_scale_y="));
    stream.println(accel_scale_y, 8);
    stream.print(F("accel_scale_z="));
    stream.println(accel_scale_z, 8);
}

void MPU6500::set_gyro_lpf(float alpha)
{
    gyro_alpha = clamp_alpha(alpha);
}

void MPU6500::set_accel_lpf(float alpha)
{
    accel_alpha = clamp_alpha(alpha);
}

void MPU6500::reset_filters()
{
    filters_initialized = false;
}

const MPU6500::raw_data_t & MPU6500::get_raw() const
{
    return raw;
}

const MPU6500::scaled_data_t & MPU6500::get_scaled() const
{
    return scaled;
}

const MPU6500::scaled_data_t & MPU6500::get_filtered() const
{
    return filtered;
}

const MPU6500::scaled_data_t & MPU6500::get_data() const
{
    return filtered;
}

const MPU6500::timing_t & MPU6500::get_timing() const
{
    return timing;
}
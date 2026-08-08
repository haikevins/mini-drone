/*
 * vl53l1x.h
 * Minimal register-level VL53L1X driver for this flight controller.
 * No third-party VL53L1X library is required.
 */

#ifndef VL53L1X_DRIVER_H
#define VL53L1X_DRIVER_H

#include <Arduino.h>
#include "hal/i2c_bus.h"

class VL53L1X
{
    public:
        static constexpr uint8_t default_i2c_address = 0x29u;

        enum class distance_mode_t : uint8_t
        {
            SHORT,
            MEDIUM,
            LONG
        };

        enum class interrupt_polarity_t : uint8_t
        {
            ACTIVE_LOW = 0u,
            ACTIVE_HIGH = 1u
        };

        enum class init_error_t : uint8_t
        {
            NONE = 0u,
            BUS_NOT_READY,
            MODEL_ID_READ,
            MODEL_ID_MISMATCH,
            SOFT_RESET_LOW,
            SOFT_RESET_HIGH,
            BOOT_TIMEOUT,
            PAD_CONFIG_READ,
            PAD_CONFIG_WRITE,
            OSC_FREQUENCY_READ,
            OSC_CALIBRATE_READ,
            STATIC_CONFIG,
            OFFSET_READ,
            OFFSET_WRITE,
            POLARITY_READ,
            DISTANCE_MODE,
            TIMING_BUDGET,
            START_CONTINUOUS
        };

        /*
         * Public range status follows the simplified status mapping used by
         * the original VL53L1X driver. RESULT__RANGE_STATUS itself contains
         * an internal/raw firmware code; for example raw 9 (RANGECOMPLETE)
         * maps to VALID when stream_count is non-zero.
         */
        enum class range_status_t : uint8_t
        {
            VALID = 0u,
            SIGMA_FAIL = 1u,
            SIGNAL_FAIL = 2u,
            VALID_MIN_RANGE_CLIPPED = 3u,
            OUT_OF_BOUNDS_FAIL = 4u,
            HARDWARE_FAIL = 5u,
            VALID_NO_WRAP_CHECK_FAIL = 6u,
            WRAP_TARGET_FAIL = 7u,
            PROCESSING_FAIL = 8u,
            XTALK_SIGNAL_FAIL = 9u,
            SYNCHRONIZATION_INT = 10u,
            MIN_RANGE_FAIL = 11u,
            NONE = 255u
        };

        struct data_t
        {
            uint16_t distance_mm = 0u;
            uint16_t height_mm = 0u;
            float height_cm = 0.0f;
            float height_m = 0.0f;

            range_status_t range_status = range_status_t::NONE;
            uint8_t raw_range_status = 0u;

            /*
             * RESULT__STREAM_COUNT is kept only as raw sensor diagnostics.
             * Freshness is tracked by new_data + sample_count instead.
             */
            uint8_t stream_count = 0u;
            uint32_t sample_count = 0u;

            /*
             * Application-level validity gate.  The ST range status is the
             * primary validity source; zero-distance and >4 m samples are
             * rejected as sanity checks for the VL53L1X.
             */
            bool range_valid = false;

            /* Useful quality diagnostics from the same 17-byte result block. */
            float peak_signal_mcps = 0.0f;
            float ambient_rate_mcps = 0.0f;
            float effective_spads = 0.0f;

            bool new_data = false;
            bool healthy = false;
            bool timeout = false;

            // Timestamp and exact interval between fresh ranging samples.
            uint32_t t_us = 0u;
            uint32_t dt_us = 0u;
            float dt = 0.0f;

            // Kept for timeout/debug compatibility with the existing code.
            uint32_t t_ms = 0u;
        };

        bool begin(I2CBus * p_bus,
                   uint8_t i2c_address = default_i2c_address,
                   uint32_t timeout_ms = 500u,
                   int16_t offset_mm = 0);

        /*
         * Non-blocking sensor service using the same contract as ADNS3080:
         *   true  = one fresh ranging sample was latched successfully
         *   false = no fresh sample on this call, or an I2C/sensor error
         *
         * Use is_healthy() / data.healthy to distinguish "not ready yet"
         * from an actual sensor/communication failure.
         */
        bool update();

        bool set_distance_mode(distance_mode_t mode);
        bool set_interrupt_polarity(interrupt_polarity_t polarity);
        bool set_measurement_timing_budget_us(uint32_t budget_us);
        bool start_continuous(uint32_t period_ms);
        bool stop_continuous();

        void set_timeout_ms(uint32_t timeout_ms);
        void set_offset_mm(int16_t offset_mm);

        const data_t & get_data() const;
        bool is_healthy() const;
        init_error_t get_init_error() const;
        const char * get_init_error_string() const;

    private:
        struct results_t
        {
            uint8_t range_status = 0u;
            uint8_t stream_count = 0u;
            uint16_t dss_actual_effective_spads = 0u;
            uint16_t ambient_count_rate = 0u;
            uint16_t raw_range_mm = 0u;
            uint16_t peak_signal_count_rate = 0u;
        };

        bool initialize_sensor();
        bool wait_for_boot();
        bool get_interrupt_polarity(interrupt_polarity_t & polarity);
        bool data_ready(bool & ready);
        bool read_results(results_t & results);
        bool setup_manual_calibration();
        bool update_dss(const results_t & results);
        bool clear_interrupt();
        static range_status_t convert_range_status(uint8_t raw_status,
                                                   uint8_t stream_count);

        bool write_u8(uint16_t reg, uint8_t value);
        bool write_u16(uint16_t reg, uint16_t value);
        bool write_u32(uint16_t reg, uint32_t value);
        bool read_u8(uint16_t reg, uint8_t & value);
        bool read_u16(uint16_t reg, uint16_t & value);
        bool read_bytes(uint16_t reg, uint8_t * p_buffer, size_t length);

        uint32_t calc_macro_period_us_q12(uint8_t vcsel_period) const;
        static uint32_t timeout_us_to_mclks(uint32_t timeout_us,
                                            uint32_t macro_period_us_q12);
        static uint16_t encode_timeout(uint32_t timeout_mclks);

        I2CBus * p_i2c_bus = nullptr;
        uint8_t address = default_i2c_address;

        uint32_t io_timeout_ms = 500u;
        int16_t range_offset_mm = 0;

        uint16_t fast_osc_frequency = 0u;
        uint16_t osc_calibrate_value = 0u;

        uint8_t saved_vhv_init = 0u;
        uint8_t saved_vhv_timeout = 0u;
        bool manual_calibration_done = false;
        bool continuous_started = false;

        uint32_t last_measurement_ms = 0u;
        uint32_t last_measurement_us = 0u;
        bool has_previous_measurement_time = false;

        uint32_t timing_budget_us = 25000u;
        distance_mode_t distance_mode = distance_mode_t::MEDIUM;
        interrupt_polarity_t interrupt_polarity = interrupt_polarity_t::ACTIVE_HIGH;
        init_error_t init_error = init_error_t::NONE;

        data_t data;
};

#endif /* VL53L1X_DRIVER_H */

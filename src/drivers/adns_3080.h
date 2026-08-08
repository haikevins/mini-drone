/*
 * adns_3080.h
 * ADNS-3080 optical-flow sensor driver.
 *
 * The driver owns only ADNS-3080 hardware access and the latest sensor data.
 * Scheduling and flight-level processing are intentionally kept in main.cpp,
 * matching the way MPU6500 data is consumed by the application.
 */

#ifndef ADNS_3080_H
#define ADNS_3080_H

#include <Arduino.h>
#include "hal/spi_bus.h"

class ADNS3080
{
    public:
        struct data_t
        {
            int8_t dx = 0;
            int8_t dy = 0;
            uint8_t motion = 0;
            uint8_t squal = 0;
            uint16_t shutter = 0;
            uint8_t max_pixel = 0;
            bool has_motion = false;
            bool overflow = false;

            // Common sample state used by sensor drivers.
            bool new_data = false;
            bool healthy = false;
            uint32_t sample_count = 0u;

            // Timestamp and exact sample interval measured with micros().
            uint32_t t_us = 0u;
            uint32_t dt_us = 0u;
            float dt = 0.0f;
        };

        bool begin(SPIBus * p_spi_bus,
                   uint8_t chip_select_pin,
                   uint8_t reset_pin,
                   int8_t npd_pin = -1);

        /*
         * Read one Motion_Burst and update the latest ADNS data.
         *
         * Common sensor update contract:
         *   true  = one fresh sample was read successfully
         *   false = no fresh sample was produced because the SPI read failed
         *
         * MOT=0 is NOT an error. Use get_data().has_motion to inspect the
         * ADNS-3080 Motion register MOT bit.
         */
        bool update();

        const data_t & get_data() const;
        bool is_healthy() const;

        /*
         * Clear the ADNS-3080 motion accumulator/register state.
         * Application-level flow accumulators live in main.cpp and must be
         * reset there as well when overflow handling requires it.
         */
        void clear_motion();

    private:
        SPIBus * p_spi_bus = nullptr;

        uint8_t chip_select_pin = 0u;
        uint8_t reset_pin = 0u;
        int8_t npd_pin = -1;

        SPISettings spi_settings = SPISettings(2000000u, MSBFIRST, SPI_MODE3);

        data_t data;

        uint32_t last_sample_us = 0u;
        bool has_previous_sample_time = false;

        void recover_serial_port();
        void write_register(uint8_t reg, uint8_t value);
        uint8_t read_register(uint8_t reg, bool motion_delay = false);
        bool read_motion_burst(data_t & adns_data);

        void hardware_reset();
        bool check_link();
        void set_1600_cpi();
        void enable_fixed_6469_fps_experimental();
};

#endif /* ADNS_3080_H */

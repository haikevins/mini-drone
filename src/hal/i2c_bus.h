/*
 * i2c_bus.h
 * Shared I2C bus abstraction for register-based devices.
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class I2CBus
{
    public:
        explicit I2CBus(TwoWire & wire = Wire);

        bool begin(int sda_pin,
                   int scl_pin,
                   uint32_t frequency_hz = 400000u);

        bool write_u8(uint8_t device_address,
                      uint16_t register_address,
                      uint8_t value);

        bool write_u16(uint8_t device_address,
                       uint16_t register_address,
                       uint16_t value);

        bool write_u32(uint8_t device_address,
                       uint16_t register_address,
                       uint32_t value);

        bool read_u8(uint8_t device_address,
                     uint16_t register_address,
                     uint8_t & value);

        bool read_u16(uint8_t device_address,
                      uint16_t register_address,
                      uint16_t & value);

        bool read_u32(uint8_t device_address,
                      uint16_t register_address,
                      uint32_t & value);

        bool read_bytes(uint8_t device_address,
                        uint16_t register_address,
                        uint8_t * p_buffer,
                        size_t length);

        bool is_initialized() const;

    private:
        bool lock_bus();
        void unlock_bus();
        bool write_bytes_locked(uint8_t device_address,
                                uint16_t register_address,
                                const uint8_t * p_data,
                                size_t length);
        bool read_bytes_locked(uint8_t device_address,
                               uint16_t register_address,
                               uint8_t * p_buffer,
                               size_t length);

        TwoWire * p_wire = nullptr;
        SemaphoreHandle_t bus_mutex = nullptr;
        bool initialized = false;
};

#endif /* I2C_BUS_H */

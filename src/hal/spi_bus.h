/*
 * spi_bus.h
 * Shared SPI bus abstraction for multiple devices.
 */

#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <Arduino.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class SPIBus
{
    public:
        SPIBus(SPIClass & spi_class = SPI);

        /*
         * Initialize the physical SPI bus once. All SPI devices share these
         * SCK/MISO/MOSI pins and use their own chip-select pin/settings.
         */
        bool begin(int sck_pin, int miso_pin, int mosi_pin);

        /*
         * Low-level transaction API for devices that require custom command
         * framing/timing (for example ADNS-3080 motion burst reads).
         *
         * begin_transaction() locks the shared bus, applies the device's own
         * SPISettings and asserts only that device's CS. end_transaction()
         * deasserts CS, ends the SPI transaction and releases the bus lock.
         */
        bool begin_transaction(uint8_t chip_select_pin, const SPISettings & settings);
        uint8_t transfer(uint8_t value);
        void end_transaction(uint8_t chip_select_pin);

        /*
         * MPU-style register helpers. The register read/write bit convention
         * is intentionally kept here because MPU6500 already uses it:
         *   write: address bit7 = 0
         *   read : address bit7 = 1
         */
        bool write_register(uint8_t chip_select_pin,
                            uint8_t reg_addr,
                            uint8_t reg_value,
                            const SPISettings & settings);

        uint8_t read_register(uint8_t chip_select_pin,
                              uint8_t reg_addr,
                              const SPISettings & settings);

        bool read_registers(uint8_t chip_select_pin,
                            uint8_t start_reg_addr,
                            uint8_t * p_buffer,
                            size_t length,
                            const SPISettings & settings);

        bool is_initialized() const;

    private:
        SPIClass * p_spi = nullptr;
        SemaphoreHandle_t bus_mutex = nullptr;
        bool initialized = false;
        bool transaction_active = false;
};

#endif /* SPI_BUS_H */

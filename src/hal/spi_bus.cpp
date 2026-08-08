/*
 * spi_bus.cpp
 * Shared SPI bus abstraction for multiple devices.
 */

#include "hal/spi_bus.h"

SPIBus::SPIBus(SPIClass & spi_class) : p_spi(&spi_class)
{}

bool SPIBus::begin(int sck_pin, int miso_pin, int mosi_pin)
{
    if (p_spi == nullptr)
    {
        return false;
    }

    if (initialized == true)
    {
        return true;
    }

    bus_mutex = xSemaphoreCreateMutex();
    if (bus_mutex == nullptr)
    {
        return false;
    }

    p_spi->begin(sck_pin, miso_pin, mosi_pin);
    initialized = true;

    return true;
}

bool SPIBus::begin_transaction(uint8_t chip_select_pin, const SPISettings & settings)
{
    if ((initialized == false) || (p_spi == nullptr) || (bus_mutex == nullptr))
    {
        return false;
    }

    if (xSemaphoreTake(bus_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    p_spi->beginTransaction(settings);
    digitalWrite(chip_select_pin, LOW);
    transaction_active = true;

    return true;
}

uint8_t SPIBus::transfer(uint8_t value)
{
    if ((transaction_active == false) || (p_spi == nullptr))
    {
        return 0u;
    }

    return p_spi->transfer(value);
}

void SPIBus::end_transaction(uint8_t chip_select_pin)
{
    if ((transaction_active == false) || (p_spi == nullptr))
    {
        return;
    }

    digitalWrite(chip_select_pin, HIGH);
    p_spi->endTransaction();
    transaction_active = false;

    if (bus_mutex != nullptr)
    {
        xSemaphoreGive(bus_mutex);
    }
}

bool SPIBus::write_register(uint8_t chip_select_pin,
                            uint8_t reg_addr,
                            uint8_t reg_value,
                            const SPISettings & settings)
{
    if (begin_transaction(chip_select_pin, settings) == false)
    {
        return false;
    }

    transfer(reg_addr & 0x7Fu);
    transfer(reg_value);

    end_transaction(chip_select_pin);
    return true;
}

uint8_t SPIBus::read_register(uint8_t chip_select_pin,
                              uint8_t reg_addr,
                              const SPISettings & settings)
{
    if (begin_transaction(chip_select_pin, settings) == false)
    {
        return 0u;
    }

    transfer(reg_addr | 0x80u);
    const uint8_t reg_value = transfer(0x00u);

    end_transaction(chip_select_pin);
    return reg_value;
}

bool SPIBus::read_registers(uint8_t chip_select_pin,
                            uint8_t start_reg_addr,
                            uint8_t * p_buffer,
                            size_t length,
                            const SPISettings & settings)
{
    if ((p_buffer == nullptr) || (length == 0u))
    {
        return false;
    }

    if (begin_transaction(chip_select_pin, settings) == false)
    {
        return false;
    }

    transfer(start_reg_addr | 0x80u);

    for (size_t byte_index = 0u; byte_index < length; ++byte_index)
    {
        p_buffer[byte_index] = transfer(0x00u);
    }

    end_transaction(chip_select_pin);
    return true;
}

bool SPIBus::is_initialized() const
{
    return initialized;
}

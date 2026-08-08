/*
 * i2c_bus.cpp
 * Shared I2C bus abstraction for register-based devices.
 */

#include "hal/i2c_bus.h"

I2CBus::I2CBus(TwoWire & wire) : p_wire(&wire)
{}

bool I2CBus::begin(int sda_pin,
                   int scl_pin,
                   uint32_t frequency_hz)
{
    if (p_wire == nullptr)
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

    if (p_wire->begin(sda_pin, scl_pin, frequency_hz) == false)
    {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = nullptr;
        return false;
    }

    initialized = true;
    return true;
}

bool I2CBus::lock_bus()
{
    if ((initialized == false) ||
        (p_wire == nullptr) ||
        (bus_mutex == nullptr))
    {
        return false;
    }

    return xSemaphoreTake(bus_mutex, portMAX_DELAY) == pdTRUE;
}

void I2CBus::unlock_bus()
{
    if (bus_mutex != nullptr)
    {
        xSemaphoreGive(bus_mutex);
    }
}

bool I2CBus::write_bytes_locked(uint8_t device_address,
                                uint16_t register_address,
                                const uint8_t * p_data,
                                size_t length)
{
    if ((p_data == nullptr) || (length == 0u))
    {
        return false;
    }

    static constexpr uint8_t max_attempts = 3u;

    for (uint8_t attempt = 0u; attempt < max_attempts; ++attempt)
    {
        p_wire->beginTransmission(device_address);
        p_wire->write(static_cast<uint8_t>(register_address >> 8));
        p_wire->write(static_cast<uint8_t>(register_address & 0xFFu));

        for (size_t i = 0u; i < length; ++i)
        {
            p_wire->write(p_data[i]);
        }

        if (p_wire->endTransmission(true) == 0u)
        {
            return true;
        }

        delayMicroseconds(100u);
    }

    return false;
}

bool I2CBus::read_bytes_locked(uint8_t device_address,
                               uint16_t register_address,
                               uint8_t * p_buffer,
                               size_t length)
{
    if ((p_buffer == nullptr) || (length == 0u))
    {
        return false;
    }

    static constexpr uint8_t max_attempts = 3u;

    for (uint8_t attempt = 0u; attempt < max_attempts; ++attempt)
    {
        p_wire->beginTransmission(device_address);
        p_wire->write(static_cast<uint8_t>(register_address >> 8));
        p_wire->write(static_cast<uint8_t>(register_address & 0xFFu));

        if (p_wire->endTransmission(false) != 0u)
        {
            delayMicroseconds(100u);
            continue;
        }

        const size_t received = p_wire->requestFrom(
            device_address,
            length,
            true
        );

        if (received != length)
        {
            while (p_wire->available() > 0)
            {
                (void)p_wire->read();
            }

            delayMicroseconds(100u);
            continue;
        }

        bool complete = true;
        for (size_t i = 0u; i < length; ++i)
        {
            if (p_wire->available() <= 0)
            {
                complete = false;
                break;
            }

            p_buffer[i] = static_cast<uint8_t>(p_wire->read());
        }

        if (complete == true)
        {
            return true;
        }

        delayMicroseconds(100u);
    }

    return false;
}

bool I2CBus::write_u8(uint8_t device_address,
                      uint16_t register_address,
                      uint8_t value)
{
    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = write_bytes_locked(
        device_address,
        register_address,
        &value,
        1u
    );

    unlock_bus();
    return ok;
}

bool I2CBus::write_u16(uint8_t device_address,
                       uint16_t register_address,
                       uint16_t value)
{
    const uint8_t data[2] =
    {
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFFu)
    };

    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = write_bytes_locked(
        device_address,
        register_address,
        data,
        sizeof(data)
    );

    unlock_bus();
    return ok;
}

bool I2CBus::write_u32(uint8_t device_address,
                       uint16_t register_address,
                       uint32_t value)
{
    const uint8_t data[4] =
    {
        static_cast<uint8_t>(value >> 24),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFFu)
    };

    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = write_bytes_locked(
        device_address,
        register_address,
        data,
        sizeof(data)
    );

    unlock_bus();
    return ok;
}

bool I2CBus::read_u8(uint8_t device_address,
                     uint16_t register_address,
                     uint8_t & value)
{
    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = read_bytes_locked(
        device_address,
        register_address,
        &value,
        1u
    );

    unlock_bus();
    return ok;
}

bool I2CBus::read_u16(uint8_t device_address,
                      uint16_t register_address,
                      uint16_t & value)
{
    uint8_t data[2] = {0u, 0u};

    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = read_bytes_locked(
        device_address,
        register_address,
        data,
        sizeof(data)
    );

    unlock_bus();

    if (ok == false)
    {
        return false;
    }

    value = (static_cast<uint16_t>(data[0]) << 8) |
            static_cast<uint16_t>(data[1]);

    return true;
}

bool I2CBus::read_u32(uint8_t device_address,
                      uint16_t register_address,
                      uint32_t & value)
{
    uint8_t data[4] = {0u, 0u, 0u, 0u};

    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = read_bytes_locked(
        device_address,
        register_address,
        data,
        sizeof(data)
    );

    unlock_bus();

    if (ok == false)
    {
        return false;
    }

    value = (static_cast<uint32_t>(data[0]) << 24) |
            (static_cast<uint32_t>(data[1]) << 16) |
            (static_cast<uint32_t>(data[2]) << 8) |
            static_cast<uint32_t>(data[3]);

    return true;
}

bool I2CBus::read_bytes(uint8_t device_address,
                        uint16_t register_address,
                        uint8_t * p_buffer,
                        size_t length)
{
    if (lock_bus() == false)
    {
        return false;
    }

    const bool ok = read_bytes_locked(
        device_address,
        register_address,
        p_buffer,
        length
    );

    unlock_bus();
    return ok;
}

bool I2CBus::is_initialized() const
{
    return initialized;
}

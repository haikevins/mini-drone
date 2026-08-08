/*
 * adns_3080.cpp
 * ADNS-3080 optical-flow sensor driver.
 *
 * Logic is intentionally kept equivalent to the original standalone source.
 * Direct calls to SPI are replaced by shared SPIBus transactions so MPU6500
 * and ADNS-3080 can safely use the same SCK/MISO/MOSI pins with independent
 * chip-select pins and independent SPI clock settings.
 */

#include "drivers/adns_3080.h"

namespace ADNS3080Reg
{
    static constexpr uint8_t PRODUCT_ID              = 0x00;
    static constexpr uint8_t REVISION_ID             = 0x01;
    static constexpr uint8_t MOTION                  = 0x02;
    static constexpr uint8_t DELTA_X                 = 0x03;
    static constexpr uint8_t DELTA_Y                 = 0x04;
    static constexpr uint8_t SQUAL                   = 0x05;
    static constexpr uint8_t PIXEL_SUM               = 0x06;
    static constexpr uint8_t MAXIMUM_PIXEL           = 0x07;
    static constexpr uint8_t CONFIGURATION_BITS      = 0x0A;
    static constexpr uint8_t EXTENDED_CONFIG         = 0x0B;
    static constexpr uint8_t DATA_OUT_LOWER          = 0x0C;
    static constexpr uint8_t DATA_OUT_UPPER          = 0x0D;
    static constexpr uint8_t SHUTTER_LOWER           = 0x0E;
    static constexpr uint8_t SHUTTER_UPPER           = 0x0F;
    static constexpr uint8_t MOTION_CLEAR            = 0x12;
    static constexpr uint8_t FRAME_CAPTURE           = 0x13;
    static constexpr uint8_t SROM_ENABLE             = 0x14;
    static constexpr uint8_t FRAME_PERIOD_MAX_LOWER  = 0x19;
    static constexpr uint8_t FRAME_PERIOD_MAX_UPPER  = 0x1A;
    static constexpr uint8_t FRAME_PERIOD_MIN_LOWER  = 0x1B;
    static constexpr uint8_t FRAME_PERIOD_MIN_UPPER  = 0x1C;
    static constexpr uint8_t SHUTTER_MAX_LOWER       = 0x1D;
    static constexpr uint8_t SHUTTER_MAX_UPPER       = 0x1E;
    static constexpr uint8_t SROM_ID                 = 0x1F;
    static constexpr uint8_t OBSERVATION             = 0x3D;
    static constexpr uint8_t INVERSE_PRODUCT_ID      = 0x3F;
    static constexpr uint8_t PIXEL_BURST             = 0x40;
    static constexpr uint8_t MOTION_BURST            = 0x50;
    static constexpr uint8_t SROM_LOAD               = 0x60;

    static constexpr uint8_t PRODUCT_ID_VALUE        = 0x17;
    static constexpr uint8_t INVERSE_PRODUCT_VALUE   = 0xE8;

    // Motion register bits
    static constexpr uint8_t MOTION_BIT              = 0x80;
    static constexpr uint8_t OVERFLOW_BIT            = 0x10;

    // Configuration_Bits
    static constexpr uint8_t CONFIG_1600_CPI         = 0x10;
}

// Datasheet timing - kept exactly from the standalone ADNS-3080 project.
static constexpr uint32_t T_SRAD_US       = 50u;
static constexpr uint32_t T_SRAD_MOT_US   = 75u;
static constexpr uint32_t T_SWW_US        = 50u;
static constexpr uint32_t T_SWR_US        = 50u;
static constexpr uint32_t T_BEXIT_US      = 5u;

bool ADNS3080::begin(SPIBus * p_spi_bus_ref,
                     uint8_t chip_select_pin_ref,
                     uint8_t reset_pin_ref,
                     int8_t npd_pin_ref)
{
    if ((p_spi_bus_ref == nullptr) || (p_spi_bus_ref->is_initialized() == false))
    {
        return false;
    }

    data = data_t{};

    p_spi_bus = p_spi_bus_ref;
    chip_select_pin = chip_select_pin_ref;
    reset_pin = reset_pin_ref;
    npd_pin = npd_pin_ref;

    data = data_t{};
    last_sample_us = 0u;
    has_previous_sample_time = false;

    pinMode(chip_select_pin, OUTPUT);
    pinMode(reset_pin, OUTPUT);

    digitalWrite(chip_select_pin, HIGH);
    digitalWrite(reset_pin, LOW);

    if (npd_pin >= 0)
    {
        pinMode(npd_pin, OUTPUT);
        digitalWrite(npd_pin, HIGH); // NPD high = normal operation
    }

    /*
     * Original code called SPI.begin(...) here. The physical bus is now
     * initialized once by main and shared by MPU6500 + ADNS-3080.
     */
    delay(10);
    hardware_reset();

    if (check_link() == false)
    {
        return false;
    }

    set_1600_cpi();
    clear_motion();

    const uint8_t rev = read_register(ADNS3080Reg::REVISION_ID);
    const uint8_t srom = read_register(ADNS3080Reg::SROM_ID);

    Serial.print("Revision ID: 0x");
    Serial.println(rev, HEX);

    Serial.print("SROM ID: 0x");
    Serial.println(srom, HEX);

    data.healthy = true;
    return true;
}

bool ADNS3080::update()
{
    data.new_data = false;

    if (read_motion_burst(data) == false)
    {
        data.healthy = false;
        return false;
    }

    data.new_data = true;
    data.healthy = true;
    data.sample_count++;
    return true;
}

void ADNS3080::recover_serial_port()
{
    digitalWrite(chip_select_pin, HIGH);
    delayMicroseconds(1000);
}

void ADNS3080::write_register(uint8_t reg, uint8_t value)
{
    if ((p_spi_bus == nullptr) ||
        (p_spi_bus->begin_transaction(chip_select_pin, spi_settings) == false))
    {
        return;
    }

    p_spi_bus->transfer(reg | 0x80u);
    p_spi_bus->transfer(value);

    p_spi_bus->end_transaction(chip_select_pin);

    delayMicroseconds(T_SWW_US);
}

uint8_t ADNS3080::read_register(uint8_t reg, bool motion_delay)
{
    if ((p_spi_bus == nullptr) ||
        (p_spi_bus->begin_transaction(chip_select_pin, spi_settings) == false))
    {
        return 0u;
    }

    p_spi_bus->transfer(reg & 0x7Fu);
    delayMicroseconds(motion_delay ? T_SRAD_MOT_US : T_SRAD_US);
    const uint8_t value = p_spi_bus->transfer(0x00u);

    p_spi_bus->end_transaction(chip_select_pin);

    return value;
}

bool ADNS3080::read_motion_burst(data_t & adns_data)
{
    uint8_t buffer[7];

    if ((p_spi_bus == nullptr) ||
        (p_spi_bus->begin_transaction(chip_select_pin, spi_settings) == false))
    {
        return false;
    }

    p_spi_bus->transfer(ADNS3080Reg::MOTION_BURST);
    delayMicroseconds(T_SRAD_MOT_US);

    // Motion_Burst order:
    // Motion, Delta_X, Delta_Y, SQUAL, Shutter_Upper, Shutter_Lower, Maximum_Pixel
    for (uint8_t i = 0u; i < sizeof(buffer); ++i)
    {
        buffer[i] = p_spi_bus->transfer(0x00u);
    }

    p_spi_bus->end_transaction(chip_select_pin);

    delayMicroseconds(T_BEXIT_US);

    adns_data.motion = buffer[0];
    adns_data.dx = static_cast<int8_t>(buffer[1]);
    adns_data.dy = static_cast<int8_t>(buffer[2]);
    adns_data.squal = buffer[3];
    adns_data.shutter = (static_cast<uint16_t>(buffer[4]) << 8) | buffer[5];
    adns_data.max_pixel = buffer[6];
    adns_data.has_motion = (adns_data.motion & ADNS3080Reg::MOTION_BIT) != 0u;
    adns_data.overflow = (adns_data.motion & ADNS3080Reg::OVERFLOW_BIT) != 0u;

    /*
     * Timestamp the completed Motion_Burst read. Unsigned subtraction keeps
     * dt_us correct across the normal micros() 32-bit wrap-around.
     */
    const uint32_t sample_time_us = micros();

    if (has_previous_sample_time == true)
    {
        adns_data.dt_us = sample_time_us - last_sample_us;
        adns_data.dt = static_cast<float>(adns_data.dt_us) * 1.0e-6f;
    }
    else
    {
        adns_data.dt_us = 0u;
        adns_data.dt = 0.0f;
        has_previous_sample_time = true;
    }

    last_sample_us = sample_time_us;
    adns_data.t_us = sample_time_us;

    return true;
}

void ADNS3080::hardware_reset()
{
    digitalWrite(chip_select_pin, HIGH);

    digitalWrite(reset_pin, HIGH);
    delayMicroseconds(20);
    digitalWrite(reset_pin, LOW);

    delay(35);

    recover_serial_port();
}

void ADNS3080::clear_motion()
{
    write_register(ADNS3080Reg::MOTION_CLEAR, 0xFFu);
}

bool ADNS3080::check_link()
{
    const uint8_t pid = read_register(ADNS3080Reg::PRODUCT_ID);
    const uint8_t ipid = read_register(ADNS3080Reg::INVERSE_PRODUCT_ID);

    Serial.print("Product ID: 0x");
    Serial.print(pid, HEX);
    Serial.print("  Inverse ID: 0x");
    Serial.println(ipid, HEX);

    return (pid == ADNS3080Reg::PRODUCT_ID_VALUE) &&
           (ipid == ADNS3080Reg::INVERSE_PRODUCT_VALUE);
}

void ADNS3080::set_1600_cpi()
{
    uint8_t config = read_register(ADNS3080Reg::CONFIGURATION_BITS);
    config |= ADNS3080Reg::CONFIG_1600_CPI;
    write_register(ADNS3080Reg::CONFIGURATION_BITS, config);

    const uint8_t motion = read_register(ADNS3080Reg::MOTION, true);
    Serial.print("Motion reg after 1600 CPI set: 0x");
    Serial.println(motion, HEX);
}

void ADNS3080::enable_fixed_6469_fps_experimental()
{
    uint8_t ext = read_register(ADNS3080Reg::EXTENDED_CONFIG);
    ext |= 0x01u;
    write_register(ADNS3080Reg::EXTENDED_CONFIG, ext);

    write_register(ADNS3080Reg::FRAME_PERIOD_MAX_LOWER, 0x7Eu);
    write_register(ADNS3080Reg::FRAME_PERIOD_MAX_UPPER, 0x0Eu);

    delay(1);
}

const ADNS3080::data_t & ADNS3080::get_data() const
{
    return data;
}

bool ADNS3080::is_healthy() const
{
    return data.healthy;
}

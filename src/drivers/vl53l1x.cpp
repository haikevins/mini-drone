/*
 * vl53l1x.cpp
 * Register-level VL53L1X driver used by the flight-controller application.
 */

#include "drivers/vl53l1x.h"

namespace VL53L1XReg
{
    static constexpr uint16_t SOFT_RESET = 0x0000u;
    static constexpr uint16_t VHV_CONFIG_TIMEOUT = 0x0008u;
    static constexpr uint16_t VHV_CONFIG_INIT = 0x000Bu;
    static constexpr uint16_t OSC_FAST_FREQUENCY = 0x0006u;

    static constexpr uint16_t ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x001Eu;
    static constexpr uint16_t MM_OUTER_OFFSET_MM = 0x0022u;
    static constexpr uint16_t DSS_TARGET_TOTAL_RATE_MCPS = 0x0024u;
    static constexpr uint16_t PAD_I2C_HV_EXTSUP_CONFIG = 0x002Eu;
    static constexpr uint16_t GPIO_HV_MUX_CTRL = 0x0030u;
    static constexpr uint16_t GPIO_TIO_HV_STATUS = 0x0031u;
    static constexpr uint16_t SIGMA_EFFECTIVE_PULSE_WIDTH_NS = 0x0036u;
    static constexpr uint16_t SIGMA_EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037u;
    static constexpr uint16_t XTALK_VALID_HEIGHT_MM = 0x0039u;
    static constexpr uint16_t RANGE_IGNORE_VALID_HEIGHT_MM = 0x003Eu;
    static constexpr uint16_t RANGE_MIN_CLIP = 0x003Fu;
    static constexpr uint16_t CONSISTENCY_CHECK_TOLERANCE = 0x0040u;

    static constexpr uint16_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0046u;
    static constexpr uint16_t CAL_CONFIG_VCSEL_START = 0x0047u;
    static constexpr uint16_t PHASECAL_TIMEOUT_MACROP = 0x004Bu;
    static constexpr uint16_t PHASECAL_OVERRIDE = 0x004Du;
    static constexpr uint16_t DSS_ROI_MODE_CONTROL = 0x004Fu;
    static constexpr uint16_t SYSTEM_THRESH_RATE_HIGH = 0x0050u;
    static constexpr uint16_t SYSTEM_THRESH_RATE_LOW = 0x0052u;
    static constexpr uint16_t DSS_MANUAL_EFFECTIVE_SPADS = 0x0054u;
    static constexpr uint16_t DSS_APERTURE_ATTENUATION = 0x0057u;
    static constexpr uint16_t MM_TIMEOUT_MACROP_A = 0x005Au;
    static constexpr uint16_t MM_TIMEOUT_MACROP_B = 0x005Cu;
    static constexpr uint16_t RANGE_TIMEOUT_MACROP_A = 0x005Eu;
    static constexpr uint16_t RANGE_VCSEL_PERIOD_A = 0x0060u;
    static constexpr uint16_t RANGE_TIMEOUT_MACROP_B = 0x0061u;
    static constexpr uint16_t RANGE_VCSEL_PERIOD_B = 0x0063u;
    static constexpr uint16_t RANGE_SIGMA_THRESH = 0x0064u;
    static constexpr uint16_t RANGE_MIN_COUNT_RATE = 0x0066u;
    static constexpr uint16_t RANGE_VALID_PHASE_HIGH = 0x0069u;
    static constexpr uint16_t SYSTEM_INTERMEASUREMENT_PERIOD = 0x006Cu;

    static constexpr uint16_t GROUPED_PARAMETER_HOLD_0 = 0x0071u;
    static constexpr uint16_t SYSTEM_SEED_CONFIG = 0x0077u;
    static constexpr uint16_t SD_WOI_SD0 = 0x0078u;
    static constexpr uint16_t SD_WOI_SD1 = 0x0079u;
    static constexpr uint16_t SD_INITIAL_PHASE_SD0 = 0x007Au;
    static constexpr uint16_t SD_INITIAL_PHASE_SD1 = 0x007Bu;
    static constexpr uint16_t GROUPED_PARAMETER_HOLD_1 = 0x007Cu;
    static constexpr uint16_t SD_QUANTIFIER = 0x007Eu;
    static constexpr uint16_t SYSTEM_SEQUENCE_CONFIG = 0x0081u;
    static constexpr uint16_t GROUPED_PARAMETER_HOLD = 0x0082u;
    static constexpr uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086u;
    static constexpr uint16_t SYSTEM_MODE_START = 0x0087u;

    static constexpr uint16_t RESULT_RANGE_STATUS = 0x0089u;
    static constexpr uint16_t PHASECAL_RESULT_VCSEL_START = 0x00D8u;
    static constexpr uint16_t RESULT_OSC_CALIBRATE_VALUE = 0x00DEu;
    static constexpr uint16_t FIRMWARE_SYSTEM_STATUS = 0x00E5u;
    static constexpr uint16_t IDENTIFICATION_MODEL_ID = 0x010Fu;
}

namespace
{
    static constexpr uint16_t expected_model_id = 0xEACCu;
    static constexpr uint16_t target_rate_mcps = 0x0A00u;
    static constexpr uint32_t timing_guard_us = 4528u;
    static constexpr uint32_t max_range_timeout_us = 1100000u;
    static constexpr uint32_t ranging_gain_numerator = 2011u;
    static constexpr uint32_t ranging_gain_denominator = 2048u;

    /* VL53L1X product maximum specified ranging distance. */
    static constexpr uint16_t max_valid_distance_mm = 4000u;
}

bool VL53L1X::begin(I2CBus * p_bus,
                    uint8_t i2c_address,
                    uint32_t timeout_ms,
                    int16_t offset_mm)
{
    init_error = init_error_t::NONE;

    if ((p_bus == nullptr) || (p_bus->is_initialized() == false))
    {
        init_error = init_error_t::BUS_NOT_READY;
        return false;
    }

    p_i2c_bus = p_bus;
    address = i2c_address;
    io_timeout_ms = timeout_ms;
    range_offset_mm = offset_mm;

    data = data_t{};
    data.healthy = false;
    manual_calibration_done = false;
    continuous_started = false;
    last_measurement_us = 0u;
    has_previous_measurement_time = false;

    if (initialize_sensor() == false)
    {
        return false;
    }

    /*
     * Do not force a new polarity here.  The original working VL53L1X
     * project leaves the sensor's reset/default polarity in place.  We read
     * the actual configured polarity and use it when polling data-ready.
     */
    interrupt_polarity_t configured_polarity;
    if (get_interrupt_polarity(configured_polarity) == false)
    {
        init_error = init_error_t::POLARITY_READ;
        return false;
    }
    interrupt_polarity = configured_polarity;

    /*
     * Flow-deck style altitude rate for this project:
     * MEDIUM mode + 25 ms timing budget + 25 ms inter-measurement period.
     * This targets approximately 40 fresh height samples per second.
     */
    if (set_distance_mode(distance_mode_t::MEDIUM) == false)
    {
        init_error = init_error_t::DISTANCE_MODE;
        return false;
    }

    if (set_measurement_timing_budget_us(25000u) == false)
    {
        init_error = init_error_t::TIMING_BUDGET;
        return false;
    }

    if (start_continuous(25u) == false)
    {
        init_error = init_error_t::START_CONTINUOUS;
        return false;
    }

    init_error = init_error_t::NONE;
    data.healthy = true;
    return true;
}

bool VL53L1X::initialize_sensor()
{
    uint16_t model_id = 0u;
    if (read_u16(VL53L1XReg::IDENTIFICATION_MODEL_ID, model_id) == false)
    {
        init_error = init_error_t::MODEL_ID_READ;
        return false;
    }

    if (model_id != expected_model_id)
    {
        init_error = init_error_t::MODEL_ID_MISMATCH;
        return false;
    }

    if (write_u8(VL53L1XReg::SOFT_RESET, 0x00u) == false)
    {
        init_error = init_error_t::SOFT_RESET_LOW;
        return false;
    }

    delayMicroseconds(100u);

    if (write_u8(VL53L1XReg::SOFT_RESET, 0x01u) == false)
    {
        init_error = init_error_t::SOFT_RESET_HIGH;
        return false;
    }

    delay(1u);

    if (wait_for_boot() == false)
    {
        init_error = init_error_t::BOOT_TIMEOUT;
        return false;
    }

    uint8_t pad_config = 0u;
    if (read_u8(VL53L1XReg::PAD_I2C_HV_EXTSUP_CONFIG, pad_config) == false)
    {
        init_error = init_error_t::PAD_CONFIG_READ;
        return false;
    }

    if (write_u8(VL53L1XReg::PAD_I2C_HV_EXTSUP_CONFIG,
                 static_cast<uint8_t>(pad_config | 0x01u)) == false)
    {
        init_error = init_error_t::PAD_CONFIG_WRITE;
        return false;
    }

    if (read_u16(VL53L1XReg::OSC_FAST_FREQUENCY, fast_osc_frequency) == false ||
        fast_osc_frequency == 0u)
    {
        init_error = init_error_t::OSC_FREQUENCY_READ;
        return false;
    }

    if (read_u16(VL53L1XReg::RESULT_OSC_CALIBRATE_VALUE,
                 osc_calibrate_value) == false ||
        osc_calibrate_value == 0u)
    {
        init_error = init_error_t::OSC_CALIBRATE_READ;
        return false;
    }

    /*
     * Follow the same low-power autonomous initialization sequence used by
     * the original working project.  GPIO__TIO_HV_STATUS (0x0031) is written
     * by that sequence, but some I2C implementations/modules can report an
     * error for this status/config byte.  The original library does not make
     * this individual write fatal, so we preserve that behavior and verify
     * bus health with subsequent accesses instead.
     */
    if (write_u16(VL53L1XReg::DSS_TARGET_TOTAL_RATE_MCPS, target_rate_mcps) == false)
    {
        init_error = init_error_t::STATIC_CONFIG;
        return false;
    }

    (void)write_u8(VL53L1XReg::GPIO_TIO_HV_STATUS, 0x02u);

    if (write_u8(VL53L1XReg::SIGMA_EFFECTIVE_PULSE_WIDTH_NS, 8u) == false ||
        write_u8(VL53L1XReg::SIGMA_EFFECTIVE_AMBIENT_WIDTH_NS, 16u) == false ||
        write_u8(VL53L1XReg::XTALK_VALID_HEIGHT_MM, 0x01u) == false ||
        write_u8(VL53L1XReg::RANGE_IGNORE_VALID_HEIGHT_MM, 0xFFu) == false ||
        write_u8(VL53L1XReg::RANGE_MIN_CLIP, 0u) == false ||
        write_u8(VL53L1XReg::CONSISTENCY_CHECK_TOLERANCE, 2u) == false ||
        write_u16(VL53L1XReg::SYSTEM_THRESH_RATE_HIGH, 0x0000u) == false ||
        write_u16(VL53L1XReg::SYSTEM_THRESH_RATE_LOW, 0x0000u) == false ||
        write_u8(VL53L1XReg::DSS_APERTURE_ATTENUATION, 0x38u) == false ||
        write_u16(VL53L1XReg::RANGE_SIGMA_THRESH, 360u) == false ||
        write_u16(VL53L1XReg::RANGE_MIN_COUNT_RATE, 192u) == false ||
        write_u8(VL53L1XReg::GROUPED_PARAMETER_HOLD_0, 0x01u) == false ||
        write_u8(VL53L1XReg::GROUPED_PARAMETER_HOLD_1, 0x01u) == false ||
        write_u8(VL53L1XReg::SD_QUANTIFIER, 2u) == false ||
        write_u8(VL53L1XReg::GROUPED_PARAMETER_HOLD, 0x00u) == false ||
        write_u8(VL53L1XReg::SYSTEM_SEED_CONFIG, 1u) == false ||
        write_u8(VL53L1XReg::SYSTEM_SEQUENCE_CONFIG, 0x8Bu) == false ||
        write_u16(VL53L1XReg::DSS_MANUAL_EFFECTIVE_SPADS,
                  static_cast<uint16_t>(200u << 8)) == false ||
        write_u8(VL53L1XReg::DSS_ROI_MODE_CONTROL, 2u) == false)
    {
        init_error = init_error_t::STATIC_CONFIG;
        return false;
    }

    uint16_t outer_offset = 0u;
    if (read_u16(VL53L1XReg::MM_OUTER_OFFSET_MM, outer_offset) == false)
    {
        init_error = init_error_t::OFFSET_READ;
        return false;
    }

    if (write_u16(VL53L1XReg::ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                  static_cast<uint16_t>(outer_offset * 4u)) == false)
    {
        init_error = init_error_t::OFFSET_WRITE;
        return false;
    }

    return true;
}

bool VL53L1X::wait_for_boot()
{
    const uint32_t start_ms = millis();

    while (true)
    {
        uint8_t firmware_status = 0u;

        if (read_u8(VL53L1XReg::FIRMWARE_SYSTEM_STATUS,
                    firmware_status) == true)
        {
            if ((firmware_status & 0x01u) != 0u)
            {
                return true;
            }
        }

        if ((millis() - start_ms) >= io_timeout_ms)
        {
            return false;
        }

        delay(1u);
    }
}

bool VL53L1X::set_distance_mode(distance_mode_t mode)
{
    uint8_t vcsel_a = 0u;
    uint8_t vcsel_b = 0u;
    uint8_t valid_phase_high = 0u;
    uint8_t initial_phase = 0u;

    switch (mode)
    {
        case distance_mode_t::SHORT:
            vcsel_a = 0x07u;
            vcsel_b = 0x05u;
            valid_phase_high = 0x38u;
            initial_phase = 6u;
            break;

        case distance_mode_t::MEDIUM:
            vcsel_a = 0x0Bu;
            vcsel_b = 0x09u;
            valid_phase_high = 0x78u;
            initial_phase = 10u;
            break;

        case distance_mode_t::LONG:
            vcsel_a = 0x0Fu;
            vcsel_b = 0x0Du;
            valid_phase_high = 0xB8u;
            initial_phase = 14u;
            break;

        default:
            return false;
    }

    if (write_u8(VL53L1XReg::RANGE_VCSEL_PERIOD_A, vcsel_a) == false ||
        write_u8(VL53L1XReg::RANGE_VCSEL_PERIOD_B, vcsel_b) == false ||
        write_u8(VL53L1XReg::RANGE_VALID_PHASE_HIGH, valid_phase_high) == false ||
        write_u8(VL53L1XReg::SD_WOI_SD0, vcsel_a) == false ||
        write_u8(VL53L1XReg::SD_WOI_SD1, vcsel_b) == false ||
        write_u8(VL53L1XReg::SD_INITIAL_PHASE_SD0, initial_phase) == false ||
        write_u8(VL53L1XReg::SD_INITIAL_PHASE_SD1, initial_phase) == false)
    {
        return false;
    }

    distance_mode = mode;

    if (timing_budget_us > 0u)
    {
        return set_measurement_timing_budget_us(timing_budget_us);
    }

    return true;
}

bool VL53L1X::set_measurement_timing_budget_us(uint32_t budget_us)
{
    if ((budget_us <= timing_guard_us) || (fast_osc_frequency == 0u))
    {
        return false;
    }

    uint32_t range_timeout_us = budget_us - timing_guard_us;
    if (range_timeout_us > max_range_timeout_us)
    {
        return false;
    }

    range_timeout_us /= 2u;

    uint8_t vcsel_period_a = 0u;
    uint8_t vcsel_period_b = 0u;

    if (read_u8(VL53L1XReg::RANGE_VCSEL_PERIOD_A, vcsel_period_a) == false ||
        read_u8(VL53L1XReg::RANGE_VCSEL_PERIOD_B, vcsel_period_b) == false)
    {
        return false;
    }

    uint32_t macro_period = calc_macro_period_us_q12(vcsel_period_a);
    if (macro_period == 0u)
    {
        return false;
    }

    uint32_t phase_timeout = timeout_us_to_mclks(1000u, macro_period);
    if (phase_timeout > 0xFFu)
    {
        phase_timeout = 0xFFu;
    }

    if (write_u8(VL53L1XReg::PHASECAL_TIMEOUT_MACROP,
                 static_cast<uint8_t>(phase_timeout)) == false ||
        write_u16(VL53L1XReg::MM_TIMEOUT_MACROP_A,
                  encode_timeout(timeout_us_to_mclks(1u, macro_period))) == false ||
        write_u16(VL53L1XReg::RANGE_TIMEOUT_MACROP_A,
                  encode_timeout(timeout_us_to_mclks(range_timeout_us,
                                                      macro_period))) == false)
    {
        return false;
    }

    macro_period = calc_macro_period_us_q12(vcsel_period_b);
    if (macro_period == 0u)
    {
        return false;
    }

    if (write_u16(VL53L1XReg::MM_TIMEOUT_MACROP_B,
                  encode_timeout(timeout_us_to_mclks(1u, macro_period))) == false ||
        write_u16(VL53L1XReg::RANGE_TIMEOUT_MACROP_B,
                  encode_timeout(timeout_us_to_mclks(range_timeout_us,
                                                      macro_period))) == false)
    {
        return false;
    }

    timing_budget_us = budget_us;
    return true;
}

bool VL53L1X::start_continuous(uint32_t period_ms)
{
    if (osc_calibrate_value == 0u)
    {
        return false;
    }

    /*
     * ST ULD uses the lower 10 bits of RESULT__OSC_CALIBRATE_VAL and
     * applies a 1.075 compensation factor when converting the requested
     * inter-measurement period in milliseconds to the register value.
     *
     * Integer form of 1.075 is used here to avoid unnecessary float math.
     */
    const uint32_t clock_pll =
        static_cast<uint32_t>(osc_calibrate_value & 0x03FFu);

    if (clock_pll == 0u)
    {
        return false;
    }

    const uint64_t scaled_period =
        static_cast<uint64_t>(clock_pll) *
        static_cast<uint64_t>(period_ms) *
        1075ULL;

    const uint32_t intermeasurement =
        static_cast<uint32_t>(scaled_period / 1000ULL);

    if (write_u32(VL53L1XReg::SYSTEM_INTERMEASUREMENT_PERIOD,
                  intermeasurement) == false ||
        clear_interrupt() == false ||
        write_u8(VL53L1XReg::SYSTEM_MODE_START, 0x40u) == false)
    {
        return false;
    }

    manual_calibration_done = false;
    continuous_started = true;
    last_measurement_ms = millis();
    last_measurement_us = 0u;
    has_previous_measurement_time = false;
    data.dt_us = 0u;
    data.dt = 0.0f;
    data.timeout = false;

    return true;
}

bool VL53L1X::stop_continuous()
{
    if (write_u8(VL53L1XReg::SYSTEM_MODE_START, 0x80u) == false)
    {
        return false;
    }

    if (manual_calibration_done == true)
    {
        if (write_u8(VL53L1XReg::VHV_CONFIG_INIT, saved_vhv_init) == false ||
            write_u8(VL53L1XReg::VHV_CONFIG_TIMEOUT, saved_vhv_timeout) == false ||
            write_u8(VL53L1XReg::PHASECAL_OVERRIDE, 0x00u) == false)
        {
            return false;
        }
    }

    manual_calibration_done = false;
    continuous_started = false;
    return true;
}

bool VL53L1X::update()
{
    data.new_data = false;

    if (continuous_started == false)
    {
        data.healthy = false;
        return false;
    }

    bool ready = false;
    if (data_ready(ready) == false)
    {
        data.healthy = false;
        return false;
    }

    if (ready == false)
    {
        if ((io_timeout_ms > 0u) &&
            ((millis() - last_measurement_ms) >= io_timeout_ms))
        {
            data.timeout = true;
            data.healthy = false;
            return false;
        }

        data.healthy = true;
        return false;
    }

    results_t results;
    if (read_results(results) == false)
    {
        data.healthy = false;
        return false;
    }

    // Timestamp the actual completed result read, not the polling call.
    const uint32_t sample_time_us = micros();

    if (manual_calibration_done == false)
    {
        if (setup_manual_calibration() == false)
        {
            data.healthy = false;
            return false;
        }

        manual_calibration_done = true;
    }

    if (update_dss(results) == false)
    {
        data.healthy = false;
        return false;
    }

    const uint32_t corrected_range =
        ((static_cast<uint32_t>(results.raw_range_mm) * ranging_gain_numerator) +
         (ranging_gain_denominator / 2u)) /
        ranging_gain_denominator;

    int32_t height_mm = static_cast<int32_t>(corrected_range) -
                        static_cast<int32_t>(range_offset_mm);

    if (height_mm < 0)
    {
        height_mm = 0;
    }

    data.distance_mm = static_cast<uint16_t>(corrected_range);
    data.height_mm = static_cast<uint16_t>(height_mm);
    data.height_cm = static_cast<float>(height_mm) / 10.0f;
    data.height_m = static_cast<float>(height_mm) / 1000.0f;
    data.raw_range_status = results.range_status;
    data.range_status = convert_range_status(
        results.range_status,
        results.stream_count
    );
    data.stream_count = results.stream_count;

    /*
     * RESULT count-rate values are 9.7 fixed-point (MCPS), while the
     * effective-SPAD count is 8.8 fixed-point.
     */
    data.peak_signal_mcps =
        static_cast<float>(results.peak_signal_count_rate) / 128.0f;
    data.ambient_rate_mcps =
        static_cast<float>(results.ambient_count_rate) / 128.0f;
    data.effective_spads =
        static_cast<float>(results.dss_actual_effective_spads) / 256.0f;

    /*
     * Do not use stream_count as a freshness/validity gate.
     * Data-ready already tells us this is a fresh measurement.
     * Status 0 (VALID) is the primary sensor validity indication.
     * Reject 0 mm because VL53L1X can occasionally report a spurious
     * zero with a range-complete status; also reject values beyond the
     * product's 4 m maximum range.
     */
    data.range_valid =
        (data.range_status == range_status_t::VALID) &&
        (data.distance_mm > 0u) &&
        (data.distance_mm <= max_valid_distance_mm);

    data.new_data = true;
    data.timeout = false;

    if (has_previous_measurement_time == true)
    {
        data.dt_us = sample_time_us - last_measurement_us;
        data.dt = static_cast<float>(data.dt_us) * 1.0e-6f;
    }
    else
    {
        data.dt_us = 0u;
        data.dt = 0.0f;
    }

    data.t_us = sample_time_us;
    data.t_ms = millis();

    if (clear_interrupt() == false)
    {
        data.new_data = false;
        data.healthy = false;
        return false;
    }

    /*
     * Software sequence number: one increment per successfully consumed
     * DATA-READY measurement.  This is the reliable freshness counter for
     * the application even if the sensor's raw stream_count is constant.
     */
    data.sample_count++;
    data.healthy = true;

    last_measurement_us = sample_time_us;
    has_previous_measurement_time = true;
    last_measurement_ms = data.t_ms;
    return true;
}

bool VL53L1X::set_interrupt_polarity(interrupt_polarity_t polarity)
{
    uint8_t gpio_mux_ctrl = 0u;

    if (read_u8(VL53L1XReg::GPIO_HV_MUX_CTRL, gpio_mux_ctrl) == false)
    {
        return false;
    }

    /*
     * ST encoding is inverted at GPIO_HV_MUX_CTRL bit 4:
     *   bit 4 = 0 -> active high
     *   bit 4 = 1 -> active low
     */
    gpio_mux_ctrl &= 0xEFu;

    if (polarity == interrupt_polarity_t::ACTIVE_LOW)
    {
        gpio_mux_ctrl |= 0x10u;
    }

    if (write_u8(VL53L1XReg::GPIO_HV_MUX_CTRL, gpio_mux_ctrl) == false)
    {
        return false;
    }

    interrupt_polarity = polarity;
    return true;
}

bool VL53L1X::get_interrupt_polarity(interrupt_polarity_t & polarity)
{
    uint8_t gpio_mux_ctrl = 0u;

    if (read_u8(VL53L1XReg::GPIO_HV_MUX_CTRL, gpio_mux_ctrl) == false)
    {
        return false;
    }

    polarity = ((gpio_mux_ctrl & 0x10u) == 0u)
        ? interrupt_polarity_t::ACTIVE_HIGH
        : interrupt_polarity_t::ACTIVE_LOW;

    return true;
}

bool VL53L1X::data_ready(bool & ready)
{
    uint8_t gpio_status = 0u;

    if (read_u8(VL53L1XReg::GPIO_TIO_HV_STATUS, gpio_status) == false)
    {
        return false;
    }

    const uint8_t status_bit = gpio_status & 0x01u;
    const uint8_t ready_level = static_cast<uint8_t>(interrupt_polarity);

    ready = (status_bit == ready_level);
    return true;
}

bool VL53L1X::read_results(results_t & results)
{
    uint8_t buffer[17] = {0u};

    if (read_bytes(VL53L1XReg::RESULT_RANGE_STATUS,
                   buffer,
                   sizeof(buffer)) == false)
    {
        return false;
    }

    /* ST ULD uses the lower five bits of RESULT__RANGE_STATUS. */
    results.range_status = static_cast<uint8_t>(buffer[0] & 0x1Fu);
    results.stream_count = buffer[2];
    results.dss_actual_effective_spads =
        (static_cast<uint16_t>(buffer[3]) << 8) |
        static_cast<uint16_t>(buffer[4]);
    results.ambient_count_rate =
        (static_cast<uint16_t>(buffer[7]) << 8) |
        static_cast<uint16_t>(buffer[8]);
    results.raw_range_mm =
        (static_cast<uint16_t>(buffer[13]) << 8) |
        static_cast<uint16_t>(buffer[14]);
    results.peak_signal_count_rate =
        (static_cast<uint16_t>(buffer[15]) << 8) |
        static_cast<uint16_t>(buffer[16]);

    return true;
}

bool VL53L1X::setup_manual_calibration()
{
    uint8_t phasecal_vcsel_start = 0u;

    if (read_u8(VL53L1XReg::VHV_CONFIG_INIT, saved_vhv_init) == false ||
        read_u8(VL53L1XReg::VHV_CONFIG_TIMEOUT, saved_vhv_timeout) == false)
    {
        return false;
    }

    if (write_u8(VL53L1XReg::VHV_CONFIG_INIT,
                 static_cast<uint8_t>(saved_vhv_init & 0x7Fu)) == false ||
        write_u8(VL53L1XReg::VHV_CONFIG_TIMEOUT,
                 static_cast<uint8_t>((saved_vhv_timeout & 0x03u) |
                                      (3u << 2))) == false ||
        write_u8(VL53L1XReg::PHASECAL_OVERRIDE, 0x01u) == false ||
        read_u8(VL53L1XReg::PHASECAL_RESULT_VCSEL_START,
                phasecal_vcsel_start) == false ||
        write_u8(VL53L1XReg::CAL_CONFIG_VCSEL_START,
                 phasecal_vcsel_start) == false)
    {
        return false;
    }

    return true;
}

bool VL53L1X::update_dss(const results_t & results)
{
    const uint16_t spad_count = results.dss_actual_effective_spads;

    if (spad_count != 0u)
    {
        uint32_t total_rate_per_spad =
            static_cast<uint32_t>(results.peak_signal_count_rate) +
            static_cast<uint32_t>(results.ambient_count_rate);

        if (total_rate_per_spad > 0xFFFFu)
        {
            total_rate_per_spad = 0xFFFFu;
        }

        total_rate_per_spad <<= 16;
        total_rate_per_spad /= spad_count;

        if (total_rate_per_spad != 0u)
        {
            uint32_t required_spads =
                (static_cast<uint32_t>(target_rate_mcps) << 16) /
                total_rate_per_spad;

            if (required_spads > 0xFFFFu)
            {
                required_spads = 0xFFFFu;
            }

            return write_u16(
                VL53L1XReg::DSS_MANUAL_EFFECTIVE_SPADS,
                static_cast<uint16_t>(required_spads)
            );
        }
    }

    return write_u16(VL53L1XReg::DSS_MANUAL_EFFECTIVE_SPADS, 0x8000u);
}

VL53L1X::range_status_t VL53L1X::convert_range_status(
    uint8_t raw_status,
    uint8_t stream_count)
{
    /*
     * Same simplified status conversion used by the original working
     * VL53L1X driver (based on ST ConvertStatusLite()).
     */
    switch (raw_status)
    {
        case 17u: // MULTCLIPFAIL
        case 2u:  // VCSELWATCHDOGTESTFAILURE
        case 1u:  // VCSELCONTINUITYTESTFAILURE
        case 3u:  // NOVHVVALUEFOUND
            return range_status_t::HARDWARE_FAIL;

        case 13u: // USERROICLIP
            return range_status_t::MIN_RANGE_FAIL;

        case 18u: // GPHSTREAMCOUNT0READY
            return range_status_t::SYNCHRONIZATION_INT;

        case 5u: // RANGEPHASECHECK
            return range_status_t::OUT_OF_BOUNDS_FAIL;

        case 4u: // MSRCNOTARGET
            return range_status_t::SIGNAL_FAIL;

        case 6u: // SIGMATHRESHOLDCHECK
            return range_status_t::SIGMA_FAIL;

        case 7u: // PHASECONSISTENCY
            return range_status_t::WRAP_TARGET_FAIL;

        case 12u: // RANGEIGNORETHRESHOLD
            return range_status_t::XTALK_SIGNAL_FAIL;

        case 8u: // MINCLIP
            return range_status_t::VALID_MIN_RANGE_CLIPPED;

        case 9u: // RANGECOMPLETE
            return (stream_count == 0u)
                ? range_status_t::VALID_NO_WRAP_CHECK_FAIL
                : range_status_t::VALID;

        default:
            return range_status_t::NONE;
    }
}

bool VL53L1X::clear_interrupt()
{
    return write_u8(VL53L1XReg::SYSTEM_INTERRUPT_CLEAR, 0x01u);
}

void VL53L1X::set_timeout_ms(uint32_t timeout_ms)
{
    io_timeout_ms = timeout_ms;
}

void VL53L1X::set_offset_mm(int16_t offset_mm)
{
    range_offset_mm = offset_mm;
}

const VL53L1X::data_t & VL53L1X::get_data() const
{
    return data;
}

bool VL53L1X::is_healthy() const
{
    return data.healthy;
}

VL53L1X::init_error_t VL53L1X::get_init_error() const
{
    return init_error;
}

const char * VL53L1X::get_init_error_string() const
{
    switch (init_error)
    {
        case init_error_t::NONE: return "none";
        case init_error_t::BUS_NOT_READY: return "i2c bus not ready";
        case init_error_t::MODEL_ID_READ: return "model id read failed";
        case init_error_t::MODEL_ID_MISMATCH: return "model id mismatch";
        case init_error_t::SOFT_RESET_LOW: return "soft reset low write failed";
        case init_error_t::SOFT_RESET_HIGH: return "soft reset high write failed";
        case init_error_t::BOOT_TIMEOUT: return "boot timeout";
        case init_error_t::PAD_CONFIG_READ: return "pad config read failed";
        case init_error_t::PAD_CONFIG_WRITE: return "pad config write failed";
        case init_error_t::OSC_FREQUENCY_READ: return "osc frequency read failed";
        case init_error_t::OSC_CALIBRATE_READ: return "osc calibrate read failed";
        case init_error_t::STATIC_CONFIG: return "static config write failed";
        case init_error_t::OFFSET_READ: return "offset read failed";
        case init_error_t::OFFSET_WRITE: return "offset write failed";
        case init_error_t::POLARITY_READ: return "interrupt polarity read failed";
        case init_error_t::DISTANCE_MODE: return "distance mode config failed";
        case init_error_t::TIMING_BUDGET: return "timing budget config failed";
        case init_error_t::START_CONTINUOUS: return "start continuous failed";
        default: return "unknown";
    }
}

bool VL53L1X::write_u8(uint16_t reg, uint8_t value)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->write_u8(address, reg, value);
}

bool VL53L1X::write_u16(uint16_t reg, uint16_t value)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->write_u16(address, reg, value);
}

bool VL53L1X::write_u32(uint16_t reg, uint32_t value)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->write_u32(address, reg, value);
}

bool VL53L1X::read_u8(uint16_t reg, uint8_t & value)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->read_u8(address, reg, value);
}

bool VL53L1X::read_u16(uint16_t reg, uint16_t & value)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->read_u16(address, reg, value);
}

bool VL53L1X::read_bytes(uint16_t reg,
                         uint8_t * p_buffer,
                         size_t length)
{
    return (p_i2c_bus != nullptr) &&
           p_i2c_bus->read_bytes(address, reg, p_buffer, length);
}

uint16_t VL53L1X::encode_timeout(uint32_t timeout_mclks)
{
    if (timeout_mclks == 0u)
    {
        return 0u;
    }

    uint32_t value = timeout_mclks - 1u;
    uint16_t exponent = 0u;

    while ((value & 0xFFFFFF00u) != 0u)
    {
        value >>= 1;
        ++exponent;
    }

    return static_cast<uint16_t>((exponent << 8) | (value & 0xFFu));
}

uint32_t VL53L1X::timeout_us_to_mclks(uint32_t timeout_us,
                                      uint32_t macro_period_us_q12)
{
    if (macro_period_us_q12 == 0u)
    {
        return 0u;
    }

    return ((timeout_us << 12) + (macro_period_us_q12 >> 1)) /
           macro_period_us_q12;
}

uint32_t VL53L1X::calc_macro_period_us_q12(uint8_t vcsel_period) const
{
    if (fast_osc_frequency == 0u)
    {
        return 0u;
    }

    const uint32_t pll_period_us_q24 =
        (static_cast<uint32_t>(1u) << 30) / fast_osc_frequency;

    const uint8_t vcsel_period_pclks =
        static_cast<uint8_t>((vcsel_period + 1u) << 1);

    uint32_t macro_period = 2304u * pll_period_us_q24;
    macro_period >>= 6;
    macro_period *= vcsel_period_pclks;
    macro_period >>= 6;

    return macro_period;
}

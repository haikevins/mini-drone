/*
 * Cooperative single-task flight scheduler for Arduino-ESP32 3.3.11.
 * The flight algorithm stays in Arduino's original loopTask (priority 1).
 * Only the place and duration of the FreeRTOS release are changed.
 */
#ifndef FLIGHT_SCHEDULER_H
#define FLIGHT_SCHEDULER_H

#include <stdint.h>

namespace flight_scheduler
{
// At 1000 Hz FreeRTOS tick rate, one tick gives idle task CPU access.
// 200 ms cadence gives more frequent idle opportunity than the stock
// Arduino 5-tick release every ~2001 ms. Validated for ESP32-C3 at 1000 Hz.
constexpr uint32_t kReleasePeriodUs = 200000u;
constexpr uint32_t kReleaseTicks = 1u;

// Wrap-safe relative deadline; no accumulated catch-up bursts after a stall.
inline bool release_due(uint32_t now_us, uint32_t &last_release_us)
{
    if (uint32_t(now_us - last_release_us) < kReleasePeriodUs)
    {
        return false;
    }
    last_release_us = now_us;
    return true;
}
} // namespace flight_scheduler

#endif /* FLIGHT_SCHEDULER_H */

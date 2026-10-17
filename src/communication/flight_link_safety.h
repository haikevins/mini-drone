#ifndef FLIGHT_LINK_SAFETY_H
#define FLIGHT_LINK_SAFETY_H

#include <stdint.h>

// Pure C++ logic: independently testable on a PC.
enum class FlightLinkReason : uint8_t
{
    OK = 0,
    WAIT_DISARM = 1,
    STICKS_NOT_NEUTRAL = 2,
    COMMAND_TIMEOUT = 3,
    HEARTBEAT_TIMEOUT = 4,
    IMU_TIMEOUT = 5
};

struct FlightLinkInputs
{
    bool command_recent;
    bool heartbeat_recent;
    bool arm_request;
    bool all_sticks_neutral;
};

struct FlightLinkResult
{
    bool armed;
    bool failsafe;
    bool entered_failsafe;
    bool just_armed;
    bool just_disarmed;
    FlightLinkReason reason;
};

class FlightLinkSafety
{
public:
    FlightLinkResult update(const FlightLinkInputs &in)
    {
        const bool prev_armed = (armed_ == true);
        const bool prev_fs = (failsafe_ == true);
        const bool link_ok = ((in.command_recent == true) && (in.heartbeat_recent == true));

        // A fresh DISARM request is always honored, even if heartbeat is missing.
        if ((in.command_recent == true) && (in.arm_request == false))
        {
            armed_ = false;
            failsafe_ = false;
            disarm_seen_ = true;
            prev_arm_request_ = false;
            reason_ = FlightLinkReason::OK;
        }
        else if (link_ok == false)
        {
            if (armed_ == true)
            {
                failsafe_ = true; // latched until fresh DISARM
                reason_ = (in.command_recent == false) ? FlightLinkReason::COMMAND_TIMEOUT
                                                       : FlightLinkReason::HEARTBEAT_TIMEOUT;
            }
            else
            {
                // Reject a pre-existing ARM=1 after boot/reconnect.
                disarm_seen_ = false;
                prev_arm_request_ = true;
                reason_ = (in.command_recent == false) ? FlightLinkReason::COMMAND_TIMEOUT
                                                       : FlightLinkReason::HEARTBEAT_TIMEOUT;
            }
        }
        else
        {
            // Fresh command and heartbeat, ARM=1.
            if ((prev_arm_request_ == false) && (armed_ == false) && (failsafe_ == false))
            {
                if (disarm_seen_ == false)
                {
                    reason_ = FlightLinkReason::WAIT_DISARM;
                }
                else if (in.all_sticks_neutral == false)
                {
                    reason_ = FlightLinkReason::STICKS_NOT_NEUTRAL;
                }
                else
                {
                    armed_ = true;
                    reason_ = FlightLinkReason::OK;
                }
            }

            prev_arm_request_ = true;

            // If failsafe is latched, keep it latched until DISARM.
        }

        return {(armed_ == true),
                (failsafe_ == true),
                ((prev_fs == false) && (failsafe_ == true)),
                ((prev_armed == false) && (armed_ == true)),
                ((prev_armed == true) && (armed_ == false)),
                reason_};
    }

    bool armed() const
    {
        return (armed_ == true);
    }

    bool failsafe() const
    {
        return (failsafe_ == true);
    }

    FlightLinkReason reason() const
    {
        return reason_;
    }

private:
    bool armed_ = false;
    bool failsafe_ = false;
    bool disarm_seen_ = false;
    bool prev_arm_request_ = true; // require a fresh DISARM -> ARM rising edge
    FlightLinkReason reason_ = FlightLinkReason::WAIT_DISARM;
};

#endif

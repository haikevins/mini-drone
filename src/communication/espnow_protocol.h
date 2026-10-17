#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <Arduino.h>

typedef struct
{
    uint32_t counter;
} heartbeat_data_packet_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_data_packet_t;

typedef struct
{
    bool arm;
    bool reset;

    bool throttle_up;
    bool throttle_down;

    bool direction_forward;
    bool direction_backward;
    bool direction_left;
    bool direction_right;
} command_data_packet_t;

// Additional 16-byte message; old attitude packet remains 12 bytes.
static constexpr uint32_t FLIGHT_STATUS_MAGIC = 0x31545346u; // "FST1" (little endian)

enum FlightStatusFlags : uint8_t
{
    FLIGHT_STATUS_ARMED = 1u << 0,
    FLIGHT_STATUS_FAILSAFE = 1u << 1,
    FLIGHT_STATUS_HB_RECENT = 1u << 2,
    FLIGHT_STATUS_CMD_RECENT = 1u << 3,
    FLIGHT_STATUS_IMU_READY = 1u << 4
};

typedef struct
{
    uint32_t magic;
    uint32_t uptime_ms;
    uint32_t command_age_ms;
    uint16_t motor_output_us; // 1000 when disarmed; current base when armed
    uint8_t flags;            // FlightStatusFlags bitmask
    uint8_t reason;           // FlightLinkReason enum value
} flight_status_packet_t;

static_assert(sizeof(flight_status_packet_t) == 16, "Unexpected flight status ABI");

#endif /* ESPNOW_PROTOCOL_H */

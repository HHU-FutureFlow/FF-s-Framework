#include "navigation_protocol.h"

#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(float) == 4u, "Navigation protocol requires 32-bit float");

static uint8_t NavigationProtocolFloatIsFinite(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return (uint8_t)((bits & 0x7F800000u) != 0x7F800000u);
}

static void NavigationProtocolWriteU16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8u);
}

static void NavigationProtocolWriteU32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8u);
    destination[2] = (uint8_t)(value >> 16u);
    destination[3] = (uint8_t)(value >> 24u);
}

uint16_t NavigationProtocolChecksum(const uint8_t *data, uint16_t length)
{
    uint16_t checksum = 0u;

    if (data == NULL)
    {
        return 0u;
    }
    for (uint16_t i = 0u; i < length; i++)
    {
        checksum = (uint16_t)(checksum + data[i]);
    }
    return checksum;
}

uint8_t NavigationProtocolDecodeCommand(const uint8_t *frame,
                                        uint16_t frame_length,
                                        NavigationVelocity_s *velocity)
{
    uint16_t received_checksum;

    if ((frame == NULL) || (velocity == NULL) ||
        (frame_length != NAVIGATION_COMMAND_FRAME_SIZE) ||
        (frame[0] != NAVIGATION_COMMAND_HEADER))
    {
        return 0u;
    }

    received_checksum = (uint16_t)frame[13] | ((uint16_t)frame[14] << 8u);
    if (received_checksum != NavigationProtocolChecksum(frame, 13u))
    {
        return 0u;
    }

    memcpy(&velocity->vx, frame + 1u, sizeof(float));
    memcpy(&velocity->vy, frame + 5u, sizeof(float));
    memcpy(&velocity->wz, frame + 9u, sizeof(float));
    if ((!NavigationProtocolFloatIsFinite(velocity->vx)) ||
        (!NavigationProtocolFloatIsFinite(velocity->vy)) ||
        (!NavigationProtocolFloatIsFinite(velocity->wz)))
    {
        return 0u;
    }
    return 1u;
}

uint8_t NavigationProtocolDecodeYawTarget(const uint8_t *frame,
                                          uint16_t frame_length,
                                          NavigationYawTarget_s *target)
{
    uint16_t received_checksum;

    if ((frame == NULL) || (target == NULL) ||
        (frame_length != NAVIGATION_YAW_TARGET_FRAME_SIZE) ||
        (frame[0] != NAVIGATION_YAW_TARGET_HEADER))
    {
        return 0u;
    }

    received_checksum = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8u);
    if (received_checksum != NavigationProtocolChecksum(frame, 5u))
    {
        return 0u;
    }

    memcpy(&target->yaw_target_rad, frame + 1u, sizeof(float));
    return NavigationProtocolFloatIsFinite(target->yaw_target_rad);
}

void NavigationProtocolPackSpeedFeedback(uint32_t sample_time_us,
                                         uint8_t flags,
                                         const NavigationVelocity_s *velocity,
                                         uint8_t frame[NAVIGATION_SPEED_FEEDBACK_FRAME_SIZE])
{
    uint16_t checksum;

    frame[0] = NAVIGATION_SPEED_FEEDBACK_HEADER;
    NavigationProtocolWriteU32(frame + 1u, sample_time_us);
    frame[5] = flags;
    memcpy(frame + 6u, &velocity->vx, sizeof(float));
    memcpy(frame + 10u, &velocity->vy, sizeof(float));
    memcpy(frame + 14u, &velocity->wz, sizeof(float));
    checksum = NavigationProtocolChecksum(frame, 18u);
    NavigationProtocolWriteU16(frame + 18u, checksum);
}

void NavigationProtocolPackTurretFeedback(uint32_t sample_time_us,
                                          uint8_t flags,
                                          float turret_yaw,
                                          uint8_t frame[NAVIGATION_TURRET_FEEDBACK_FRAME_SIZE])
{
    uint16_t checksum;

    frame[0] = NAVIGATION_TURRET_FEEDBACK_HEADER;
    NavigationProtocolWriteU32(frame + 1u, sample_time_us);
    frame[5] = flags;
    memcpy(frame + 6u, &turret_yaw, sizeof(float));
    checksum = NavigationProtocolChecksum(frame, 10u);
    NavigationProtocolWriteU16(frame + 10u, checksum);
}

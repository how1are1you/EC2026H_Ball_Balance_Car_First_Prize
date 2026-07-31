#include "vision_protocol.h"

#include <stddef.h>
#include <string.h>

#define VISION_PROTOCOL_FLOAT_EXPONENT_MASK (0x7F800000UL)

static uint32_t read_u32_le(const uint8_t *bytes)
{
    return
        ((uint32_t)bytes[0]) |
        ((uint32_t)bytes[1] << 8U) |
        ((uint32_t)bytes[2] << 16U) |
        ((uint32_t)bytes[3] << 24U);
}

uint8_t vision_protocol_decode(
    const uint8_t payload[VISION_PROTOCOL_PAYLOAD_SIZE],
    float *position_mm,
    float *velocity_mm_s)
{
    uint32_t raw_position;
    uint32_t raw_velocity;
    float position_cm;
    float velocity_cm_s;
    float decoded_position_mm;
    float decoded_velocity_mm_s;

    if (payload == NULL ||
        position_mm == NULL ||
        velocity_mm_s == NULL)
    {
        return 0U;
    }

    raw_position = read_u32_le(payload);
    raw_velocity = read_u32_le(payload + 4U);
    if ((raw_position & VISION_PROTOCOL_FLOAT_EXPONENT_MASK) ==
            VISION_PROTOCOL_FLOAT_EXPONENT_MASK ||
        (raw_velocity & VISION_PROTOCOL_FLOAT_EXPONENT_MASK) ==
            VISION_PROTOCOL_FLOAT_EXPONENT_MASK)
    {
        return 0U;
    }

    memcpy(&position_cm, &raw_position, sizeof(position_cm));
    memcpy(&velocity_cm_s, &raw_velocity, sizeof(velocity_cm_s));
    decoded_position_mm = position_cm * 10.0f;
    decoded_velocity_mm_s = velocity_cm_s * 10.0f;
    if (decoded_position_mm <
            -VISION_PROTOCOL_POSITION_LIMIT_MM ||
        decoded_position_mm >
            VISION_PROTOCOL_POSITION_LIMIT_MM ||
        decoded_velocity_mm_s <
            -VISION_PROTOCOL_VELOCITY_LIMIT_MM_S ||
        decoded_velocity_mm_s >
            VISION_PROTOCOL_VELOCITY_LIMIT_MM_S)
    {
        return 0U;
    }

    *position_mm = decoded_position_mm;
    *velocity_mm_s = decoded_velocity_mm_s;
    return 1U;
}

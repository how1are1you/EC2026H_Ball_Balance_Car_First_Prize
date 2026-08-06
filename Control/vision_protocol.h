#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_PROTOCOL_PAYLOAD_SIZE (8U)
#define VISION_PROTOCOL_POSITION_LIMIT_MM (150.0f)
#define VISION_PROTOCOL_VELOCITY_LIMIT_MM_S (500.0f)

uint8_t vision_protocol_decode(
    const uint8_t payload[VISION_PROTOCOL_PAYLOAD_SIZE],
    float *position_mm,
    float *velocity_mm_s);

#endif

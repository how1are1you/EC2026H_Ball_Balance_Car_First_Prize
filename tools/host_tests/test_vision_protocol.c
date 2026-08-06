#include "vision_protocol.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

int main(void)
{
    const uint8_t valid_payload[8] = {
        0x00U, 0x00U, 0xA0U, 0x3FU,
        0x00U, 0x00U, 0x20U, 0xC0U
    };
    const uint8_t nan_payload[8] = {
        0x00U, 0x00U, 0xC0U, 0x7FU,
        0x00U, 0x00U, 0x00U, 0x00U
    };
    const uint8_t out_of_range_payload[8] = {
        0x00U, 0x00U, 0xA0U, 0x41U,
        0x00U, 0x00U, 0x00U, 0x00U
    };
    const uint8_t excessive_velocity_payload[8] = {
        0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x70U, 0x42U
    };
    float position_mm = 0.0f;
    float velocity_mm_s = 0.0f;

    assert(vision_protocol_decode(
               valid_payload,
               &position_mm,
               &velocity_mm_s) == 1U);
    assert(fabsf(position_mm - 12.5f) < 0.001f);
    assert(fabsf(velocity_mm_s + 25.0f) < 0.001f);
    assert(vision_protocol_decode(
               nan_payload,
               &position_mm,
               &velocity_mm_s) == 0U);
    assert(vision_protocol_decode(
               out_of_range_payload,
               &position_mm,
               &velocity_mm_s) == 0U);
    assert(vision_protocol_decode(
               excessive_velocity_payload,
               &position_mm,
               &velocity_mm_s) == 0U);
    assert(vision_protocol_decode(
               NULL,
               &position_mm,
               &velocity_mm_s) == 0U);

    return 0;
}

#ifndef BALL_BRAKING_H
#define BALL_BRAKING_H

#include <stdint.h>

typedef struct
{
    float stopping_distance_mm;
    float safe_velocity_mm_s;
    uint8_t moving_toward_target;
    uint8_t braking_required;
} ball_braking_result_t;

uint8_t ball_braking_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float stopping_acceleration_mm_s2,
    float delay_s,
    float margin_mm,
    float release_velocity_mm_s,
    ball_braking_result_t *result);

#endif

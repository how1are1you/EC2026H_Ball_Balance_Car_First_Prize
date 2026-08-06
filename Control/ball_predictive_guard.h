#ifndef BALL_PREDICTIVE_GUARD_H
#define BALL_PREDICTIVE_GUARD_H

#include <stdint.h>

typedef struct
{
    float predicted_error_mm;
    float guard_velocity_mm_s;
    uint8_t active;
} ball_predictive_guard_result_t;

uint8_t ball_predictive_guard_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float prediction_time_s,
    float soft_boundary_mm,
    float guard_gain_per_s,
    float guard_velocity_max_mm_s,
    ball_predictive_guard_result_t *result);

#endif

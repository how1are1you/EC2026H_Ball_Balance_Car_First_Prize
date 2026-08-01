#ifndef BALL_BRAKING_H
#define BALL_BRAKING_H

#include <stdint.h>

uint8_t ball_braking_calculate_distance_velocity_limit(
    float position_error_mm,
    float profile_acceleration_mm_s2,
    float delay_s,
    float *velocity_limit_mm_s);

#endif

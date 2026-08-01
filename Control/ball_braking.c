#include "ball_braking.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define BALL_BRAKING_POSITION_LIMIT_MM (150.0f)
#define BALL_BRAKING_VELOCITY_LIMIT_MM_S (500.0f)
#define BALL_BRAKING_ACCELERATION_LIMIT_MM_S2 (3000.0f)
#define BALL_BRAKING_DELAY_LIMIT_S (1.0f)

static uint8_t finite_float(float value)
{
    return (value == value &&
            value <= FLT_MAX &&
            value >= -FLT_MAX)
               ? 1U
               : 0U;
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

uint8_t ball_braking_calculate_distance_velocity_limit(
    float position_error_mm,
    float profile_acceleration_mm_s2,
    float delay_s,
    float *velocity_limit_mm_s)
{
    float distance_mm;
    float acceleration_delay_mm_s;
    float velocity_limit;

    if (velocity_limit_mm_s == NULL)
    {
        return 0U;
    }
    *velocity_limit_mm_s = 0.0f;

    if (finite_float(position_error_mm) == 0U ||
        finite_float(profile_acceleration_mm_s2) == 0U ||
        finite_float(delay_s) == 0U ||
        profile_acceleration_mm_s2 <= 0.0f ||
        profile_acceleration_mm_s2 >
            BALL_BRAKING_ACCELERATION_LIMIT_MM_S2 ||
        delay_s < 0.0f ||
        delay_s > BALL_BRAKING_DELAY_LIMIT_S)
    {
        return 0U;
    }

    distance_mm = clamp_float(
        absolute_float(position_error_mm),
        0.0f,
        BALL_BRAKING_POSITION_LIMIT_MM);
    acceleration_delay_mm_s =
        profile_acceleration_mm_s2 * delay_s;
    velocity_limit = sqrtf(
                         acceleration_delay_mm_s *
                             acceleration_delay_mm_s +
                         2.0f *
                             profile_acceleration_mm_s2 *
                             distance_mm) -
                     acceleration_delay_mm_s;
    *velocity_limit_mm_s = clamp_float(
        velocity_limit,
        0.0f,
        BALL_BRAKING_VELOCITY_LIMIT_MM_S);
    return 1U;
}

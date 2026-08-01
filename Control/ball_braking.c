#include "ball_braking.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define BALL_BRAKING_POSITION_LIMIT_MM (150.0f)
#define BALL_BRAKING_VELOCITY_LIMIT_MM_S (500.0f)
#define BALL_BRAKING_ACCELERATION_LIMIT_MM_S2 (3000.0f)
#define BALL_BRAKING_DELAY_LIMIT_S (1.0f)
#define BALL_BRAKING_MARGIN_LIMIT_MM (50.0f)
#define BALL_BRAKING_RELEASE_LIMIT_MM_S (100.0f)

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

uint8_t ball_braking_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float stopping_acceleration_mm_s2,
    float delay_s,
    float margin_mm,
    float release_velocity_mm_s,
    ball_braking_result_t *result)
{
    float absolute_error_mm;
    float speed_mm_s;
    float available_distance_mm;

    if (result == NULL)
    {
        return 0U;
    }

    result->stopping_distance_mm = 0.0f;
    result->safe_velocity_mm_s = 0.0f;
    result->moving_toward_target = 0U;
    result->braking_required = 0U;

    if (finite_float(position_error_mm) == 0U ||
        finite_float(velocity_mm_s) == 0U ||
        finite_float(stopping_acceleration_mm_s2) == 0U ||
        finite_float(delay_s) == 0U ||
        finite_float(margin_mm) == 0U ||
        finite_float(release_velocity_mm_s) == 0U ||
        stopping_acceleration_mm_s2 <= 0.0f ||
        stopping_acceleration_mm_s2 >
            BALL_BRAKING_ACCELERATION_LIMIT_MM_S2 ||
        delay_s < 0.0f ||
        delay_s > BALL_BRAKING_DELAY_LIMIT_S ||
        margin_mm < 0.0f ||
        margin_mm > BALL_BRAKING_MARGIN_LIMIT_MM ||
        release_velocity_mm_s < 0.0f ||
        release_velocity_mm_s >
            BALL_BRAKING_RELEASE_LIMIT_MM_S)
    {
        return 0U;
    }

    absolute_error_mm = clamp_float(
        absolute_float(position_error_mm),
        0.0f,
        BALL_BRAKING_POSITION_LIMIT_MM);
    speed_mm_s = clamp_float(
        absolute_float(velocity_mm_s),
        0.0f,
        BALL_BRAKING_VELOCITY_LIMIT_MM_S);

    result->stopping_distance_mm =
        speed_mm_s * speed_mm_s /
            (2.0f * stopping_acceleration_mm_s2) +
        speed_mm_s * delay_s +
        margin_mm;

    available_distance_mm =
        absolute_error_mm -
        speed_mm_s * delay_s -
        margin_mm;
    if (available_distance_mm > 0.0f)
    {
        result->safe_velocity_mm_s = sqrtf(
            2.0f * stopping_acceleration_mm_s2 *
            available_distance_mm);
    }

    result->moving_toward_target =
        ((position_error_mm > 0.0f && velocity_mm_s > 0.0f) ||
         (position_error_mm < 0.0f && velocity_mm_s < 0.0f))
            ? 1U
            : 0U;
    result->braking_required =
        (result->moving_toward_target != 0U &&
         speed_mm_s > release_velocity_mm_s &&
         absolute_error_mm <= result->stopping_distance_mm)
            ? 1U
            : 0U;
    return 1U;
}

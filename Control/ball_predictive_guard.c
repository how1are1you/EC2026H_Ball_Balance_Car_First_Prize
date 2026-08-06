#include "ball_predictive_guard.h"

#include <stddef.h>

#define BALL_PREDICTIVE_GUARD_POSITION_LIMIT_MM (150.0f)
#define BALL_PREDICTIVE_GUARD_VELOCITY_LIMIT_MM_S (500.0f)
#define BALL_PREDICTIVE_GUARD_PREDICTION_LIMIT_S (1.0f)
#define BALL_PREDICTIVE_GUARD_BOUNDARY_LIMIT_MM (150.0f)
#define BALL_PREDICTIVE_GUARD_GAIN_LIMIT_PER_S (100.0f)
#define BALL_PREDICTIVE_GUARD_OUTPUT_LIMIT_MM_S (500.0f)

static uint8_t finite_float(float value)
{
    return
        (value == value &&
         value <= 3.402823466e+38F &&
         value >= -3.402823466e+38F)
            ? 1U : 0U;
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

uint8_t ball_predictive_guard_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float prediction_time_s,
    float soft_boundary_mm,
    float guard_gain_per_s,
    float guard_velocity_max_mm_s,
    ball_predictive_guard_result_t *result)
{
    float excess_mm;
    float guard_magnitude_mm_s;

    if (result == NULL)
    {
        return 0U;
    }

    result->predicted_error_mm = 0.0f;
    result->guard_velocity_mm_s = 0.0f;
    result->active = 0U;

    if (finite_float(position_error_mm) == 0U ||
        finite_float(velocity_mm_s) == 0U ||
        finite_float(prediction_time_s) == 0U ||
        finite_float(soft_boundary_mm) == 0U ||
        finite_float(guard_gain_per_s) == 0U ||
        finite_float(guard_velocity_max_mm_s) == 0U ||
        absolute_float(position_error_mm) >
            BALL_PREDICTIVE_GUARD_POSITION_LIMIT_MM ||
        absolute_float(velocity_mm_s) >
            BALL_PREDICTIVE_GUARD_VELOCITY_LIMIT_MM_S ||
        prediction_time_s < 0.0f ||
        prediction_time_s >
            BALL_PREDICTIVE_GUARD_PREDICTION_LIMIT_S ||
        soft_boundary_mm < 0.0f ||
        soft_boundary_mm >
            BALL_PREDICTIVE_GUARD_BOUNDARY_LIMIT_MM ||
        guard_gain_per_s < 0.0f ||
        guard_gain_per_s >
            BALL_PREDICTIVE_GUARD_GAIN_LIMIT_PER_S ||
        guard_velocity_max_mm_s < 0.0f ||
        guard_velocity_max_mm_s >
            BALL_PREDICTIVE_GUARD_OUTPUT_LIMIT_MM_S)
    {
        return 0U;
    }

    result->predicted_error_mm =
        position_error_mm -
        velocity_mm_s * prediction_time_s;
    excess_mm =
        absolute_float(result->predicted_error_mm) -
        soft_boundary_mm;
    if (excess_mm <= 0.0f ||
        guard_gain_per_s == 0.0f ||
        guard_velocity_max_mm_s == 0.0f)
    {
        return 1U;
    }

    guard_magnitude_mm_s = clamp_float(
        guard_gain_per_s * excess_mm,
        0.0f,
        guard_velocity_max_mm_s);
    result->guard_velocity_mm_s =
        (result->predicted_error_mm < 0.0f)
            ? -guard_magnitude_mm_s
            : guard_magnitude_mm_s;
    result->active = 1U;
    return 1U;
}

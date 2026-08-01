#include "ball_predictive_guard.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void assert_guard(
    float position_error_mm,
    float velocity_mm_s,
    float expected_predicted_error_mm,
    float expected_guard_velocity_mm_s,
    uint8_t expected_active)
{
    ball_predictive_guard_result_t result;

    assert(ball_predictive_guard_calculate(
               position_error_mm,
               velocity_mm_s,
               0.060f,
               5.0f,
               8.0f,
               60.0f,
               &result) == 1U);
    assert_close(
        result.predicted_error_mm,
        expected_predicted_error_mm);
    assert_close(
        result.guard_velocity_mm_s,
        expected_guard_velocity_mm_s);
    assert(result.active == expected_active);
}

int main(void)
{
    ball_predictive_guard_result_t result;

    assert_guard(0.0f, 0.0f, 0.0f, 0.0f, 0U);
    assert_guard(-6.0f, 50.0f, -9.0f, -32.0f, 1U);
    assert_guard(-6.0f, -50.0f, -3.0f, 0.0f, 0U);
    assert_guard(6.0f, -50.0f, 9.0f, 32.0f, 1U);
    assert_guard(-8.0f, 500.0f, -38.0f, -60.0f, 1U);

    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               5.0f,
               0.0f,
               60.0f,
               &result) == 1U);
    assert_close(result.guard_velocity_mm_s, 0.0f);
    assert(result.active == 0U);

    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               5.0f,
               8.0f,
               60.0f,
               NULL) == 0U);
    assert(ball_predictive_guard_calculate(
               NAN,
               0.0f,
               0.060f,
               5.0f,
               8.0f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               NAN,
               0.060f,
               5.0f,
               8.0f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               -0.001f,
               5.0f,
               8.0f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               1.001f,
               5.0f,
               8.0f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               -0.001f,
               8.0f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               5.0f,
               -0.001f,
               60.0f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               5.0f,
               8.0f,
               -0.001f,
               &result) == 0U);
    assert(ball_predictive_guard_calculate(
               0.0f,
               0.0f,
               0.060f,
               5.0f,
               8.0f,
               500.001f,
               &result) == 0U);

    return 0;
}

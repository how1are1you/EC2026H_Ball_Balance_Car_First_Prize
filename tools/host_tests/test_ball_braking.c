#include "ball_braking.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.01f);
}

int main(void)
{
    ball_braking_result_t result;

    assert(ball_braking_calculate(
               100.0f, 0.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 3.0f);
    assert(result.moving_toward_target == 0U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 29.133f);
    assert_close(result.safe_velocity_mm_s, 60.498f);
    assert(result.moving_toward_target == 1U);
    assert(result.braking_required == 1U);

    assert(ball_braking_calculate(
               30.0f, 60.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 18.6f);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, -80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.moving_toward_target == 0U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               -20.0f, -80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.moving_toward_target == 1U);
    assert(result.braking_required == 1U);

    assert(ball_braking_calculate(
               2.0f, 8.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, 80.0f, 0.0f, 0.060f, 3.0f,
               10.0f, &result) == 0U);
    assert(ball_braking_calculate(
               NAN, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 0U);
    assert(ball_braking_calculate(
               20.0f, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, NULL) == 0U);
    return 0;
}

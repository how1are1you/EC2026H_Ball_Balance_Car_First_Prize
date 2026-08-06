#include "ball_braking.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    float velocity_limit_mm_s;

    assert(ball_braking_calculate_distance_velocity_limit(
               0.0f,
               30.0f,
               0.060f,
               &velocity_limit_mm_s) == 1U);
    assert_close(velocity_limit_mm_s, 0.0f);

    assert(ball_braking_calculate_distance_velocity_limit(
               20.0f,
               30.0f,
               0.060f,
               &velocity_limit_mm_s) == 1U);
    assert_close(velocity_limit_mm_s, 32.88775f);

    assert(ball_braking_calculate_distance_velocity_limit(
               -20.0f,
               30.0f,
               0.060f,
               &velocity_limit_mm_s) == 1U);
    assert_close(velocity_limit_mm_s, 32.88775f);

    assert(ball_braking_calculate_distance_velocity_limit(
               100.0f,
               30.0f,
               0.060f,
               &velocity_limit_mm_s) == 1U);
    assert_close(velocity_limit_mm_s, 75.68058f);

    assert(ball_braking_calculate_distance_velocity_limit(
               20.0f,
               30.0f,
               0.0f,
               &velocity_limit_mm_s) == 1U);
    assert_close(velocity_limit_mm_s, 34.64102f);

    assert(ball_braking_calculate_distance_velocity_limit(
               20.0f,
               0.0f,
               0.060f,
               &velocity_limit_mm_s) == 0U);
    assert(ball_braking_calculate_distance_velocity_limit(
               20.0f,
               30.0f,
               -0.001f,
               &velocity_limit_mm_s) == 0U);
    assert(ball_braking_calculate_distance_velocity_limit(
               NAN,
               30.0f,
               0.060f,
               &velocity_limit_mm_s) == 0U);
    assert(ball_braking_calculate_distance_velocity_limit(
               20.0f,
               30.0f,
               0.060f,
               NULL) == 0U);
    return 0;
}

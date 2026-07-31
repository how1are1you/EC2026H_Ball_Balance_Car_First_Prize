#include "ball_state_observer.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    ball_state_observer_t observer;
    ball_vision_measurement_t measurement = {
        0.0f, 100.0f, 1U, 0U, 1U
    };

    ball_state_observer_reset(&observer);
    assert(observer.valid == 0U);

    ball_state_observer_update(&observer, &measurement, 0U);
    assert(observer.valid == 1U);
    assert_close(observer.position_mm, 0.0f);
    assert_close(observer.velocity_mm_s, 100.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);

    ball_state_observer_update(&observer, &measurement, 5U);
    assert_close(observer.position_mm, 0.5f);
    assert_close(observer.velocity_mm_s, 100.0f);

    ball_state_observer_update(&observer, &measurement, 10U);
    ball_state_observer_update(&observer, &measurement, 15U);
    measurement.position_mm = 4.0f;
    measurement.frame_count = 2U;
    measurement.sample_ms = 20U;
    ball_state_observer_update(&observer, &measurement, 20U);
    assert_close(observer.position_mm, 3.1f);
    assert_close(observer.velocity_mm_s, 110.0f);
    assert_close(observer.acceleration_mm_s2, 400.0f);

    ball_state_observer_update(&observer, &measurement, 25U);
    assert_close(observer.position_mm, 3.65f);
    assert_close(observer.velocity_mm_s, 110.0f);
    assert_close(observer.acceleration_mm_s2, 320.0f);

    ball_state_observer_update(&observer, &measurement, 221U);
    assert(observer.valid == 0U);
    assert_close(observer.velocity_mm_s, 0.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);

    ball_state_observer_update(&observer, &measurement, 222U);
    assert(observer.valid == 0U);

    measurement.position_mm = 0.0f;
    measurement.velocity_mm_s = 700.0f;
    measurement.frame_count = 3U;
    measurement.sample_ms = 225U;
    ball_state_observer_update(&observer, &measurement, 225U);
    assert(observer.valid == 1U);
    assert_close(
        observer.velocity_mm_s,
        BALL_OBSERVER_VELOCITY_LIMIT_MM_S);

    measurement.position_mm = -150.0f;
    measurement.frame_count = 4U;
    measurement.sample_ms = 230U;
    ball_state_observer_update(&observer, &measurement, 230U);
    assert(fabsf(observer.acceleration_mm_s2) <=
           BALL_OBSERVER_ACCEL_LIMIT_MM_S2);

    ball_state_observer_reset(NULL);
    ball_state_observer_update(NULL, &measurement, 235U);
    ball_state_observer_update(&observer, NULL, 235U);
    assert(observer.valid == 0U);

    return 0;
}

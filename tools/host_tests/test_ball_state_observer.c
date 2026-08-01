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
    ball_state_observer_t fused_observer;
    ball_state_observer_t invalid_initial_observer;
    ball_vision_measurement_t measurement = {
        0.0f, 100.0f, 1U, 0U, 1U
    };
    ball_vision_measurement_t fused_measurement = {
        0.0f, 0.0f, 1U, 0U, 1U
    };
    ball_vision_measurement_t invalid_initial_measurement = {
        0.0f, NAN, 1U, 0U, 1U
    };

    ball_state_observer_reset(&observer);
    assert(observer.valid == 0U);
    assert_close(observer.camera_velocity_mm_s, 0.0f);
    assert(observer.velocity_blend_active == 0U);

    ball_state_observer_update(&observer, &measurement, 0U);
    assert(observer.valid == 1U);
    assert_close(observer.position_mm, 0.0f);
    assert_close(observer.velocity_mm_s, 100.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);
    assert_close(observer.camera_velocity_mm_s, 100.0f);
    assert(observer.velocity_blend_active == 0U);

    ball_state_observer_update(&observer, &measurement, 5U);
    assert_close(observer.position_mm, 0.5f);
    assert_close(observer.velocity_mm_s, 100.0f);
    assert(observer.velocity_blend_active == 0U);

    ball_state_observer_update(&observer, &measurement, 10U);
    ball_state_observer_update(&observer, &measurement, 15U);
    measurement.position_mm = 4.0f;
    measurement.frame_count = 2U;
    measurement.sample_ms = 20U;
    ball_state_observer_update(&observer, &measurement, 20U);
    assert_close(observer.position_mm, 3.1f);
    assert_close(observer.velocity_mm_s, 107.5f);
    assert_close(observer.camera_velocity_mm_s, 100.0f);
    assert(observer.velocity_blend_active == 1U);
    assert_close(observer.acceleration_mm_s2, 300.0f);

    ball_state_observer_update(&observer, &measurement, 25U);
    assert_close(observer.position_mm, 3.6375f);
    assert_close(observer.velocity_mm_s, 107.5f);
    assert(observer.velocity_blend_active == 0U);
    assert_close(observer.acceleration_mm_s2, 240.0f);

    ball_state_observer_update(&observer, &measurement, 221U);
    assert(observer.valid == 0U);
    assert_close(observer.velocity_mm_s, 0.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);
    assert_close(observer.camera_velocity_mm_s, 0.0f);
    assert(observer.velocity_blend_active == 0U);

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
    assert_close(
        observer.camera_velocity_mm_s,
        BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
    assert(observer.velocity_blend_active == 0U);

    measurement.position_mm = -150.0f;
    measurement.frame_count = 4U;
    measurement.sample_ms = 230U;
    ball_state_observer_update(&observer, &measurement, 230U);
    assert(fabsf(observer.acceleration_mm_s2) <=
           BALL_OBSERVER_ACCEL_LIMIT_MM_S2);

    ball_state_observer_reset(&fused_observer);
    ball_state_observer_update(
        &fused_observer,
        &fused_measurement,
        0U);
    fused_measurement.position_mm = 2.0f;
    fused_measurement.velocity_mm_s = 50.0f;
    fused_measurement.frame_count = 2U;
    fused_measurement.sample_ms = 20U;
    ball_state_observer_update(
        &fused_observer,
        &fused_measurement,
        20U);
    assert_close(fused_observer.position_mm, 1.1f);
    assert_close(fused_observer.velocity_mm_s, 20.0f);
    assert_close(fused_observer.camera_velocity_mm_s, 50.0f);
    assert(fused_observer.velocity_blend_active == 1U);

    ball_state_observer_update(
        &fused_observer,
        &fused_measurement,
        25U);
    assert_close(fused_observer.position_mm, 1.2f);
    assert_close(fused_observer.velocity_mm_s, 20.0f);
    assert(fused_observer.velocity_blend_active == 0U);

    fused_measurement.position_mm = 3.0f;
    fused_measurement.velocity_mm_s = NAN;
    fused_measurement.frame_count = 3U;
    fused_measurement.sample_ms = 40U;
    ball_state_observer_update(
        &fused_observer,
        &fused_measurement,
        40U);
    assert(
        fused_observer.velocity_mm_s ==
        fused_observer.velocity_mm_s);
    assert_close(fused_observer.camera_velocity_mm_s, 0.0f);
    assert(fused_observer.velocity_blend_active == 0U);

    ball_state_observer_reset(&invalid_initial_observer);
    ball_state_observer_update(
        &invalid_initial_observer,
        &invalid_initial_measurement,
        0U);
    assert_close(invalid_initial_observer.velocity_mm_s, 0.0f);
    assert_close(
        invalid_initial_observer.camera_velocity_mm_s,
        0.0f);
    assert(invalid_initial_observer.velocity_blend_active == 0U);

    ball_state_observer_reset(NULL);
    ball_state_observer_update(NULL, &measurement, 235U);
    ball_state_observer_update(&observer, NULL, 235U);
    assert(observer.valid == 0U);

    return 0;
}

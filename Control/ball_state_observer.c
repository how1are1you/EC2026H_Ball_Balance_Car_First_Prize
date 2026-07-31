#include "ball_state_observer.h"

#include <stddef.h>

ball_state_observer_t ball_state_observer;

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

static void invalidate_dynamic_state(ball_state_observer_t *observer)
{
    observer->velocity_mm_s = 0.0f;
    observer->acceleration_mm_s2 = 0.0f;
    observer->previous_velocity_mm_s = 0.0f;
    observer->initialized = 0U;
    observer->valid = 0U;
}

void ball_state_observer_reset(ball_state_observer_t *observer)
{
    if (observer == NULL)
    {
        return;
    }

    observer->position_mm = 0.0f;
    observer->velocity_mm_s = 0.0f;
    observer->acceleration_mm_s2 = 0.0f;
    observer->last_frame_count = 0U;
    observer->last_measurement_ms = 0U;
    observer->update_count = 0U;
    observer->initialized = 0U;
    observer->valid = 0U;
    observer->previous_velocity_mm_s = 0.0f;
}

void ball_state_observer_update(
    ball_state_observer_t *observer,
    const ball_vision_measurement_t *measurement,
    uint32_t now_ms)
{
    uint32_t frame_dt_ms;
    uint8_t measurement_fresh;
    uint8_t new_frame;
    uint8_t initialized_this_tick = 0U;
    float residual_mm;
    float acceleration_raw_mm_s2;

    if (observer == NULL)
    {
        return;
    }
    if (measurement == NULL)
    {
        invalidate_dynamic_state(observer);
        observer->update_count++;
        return;
    }

    measurement_fresh =
        (measurement->valid != 0U) &&
        ((uint32_t)(now_ms - measurement->sample_ms) <=
         BALL_OBSERVER_VISION_TIMEOUT_MS);
    new_frame =
        measurement_fresh != 0U &&
        measurement->frame_count != observer->last_frame_count;

    if (observer->initialized != 0U)
    {
        observer->position_mm = clamp_float(
            observer->position_mm +
                observer->velocity_mm_s * BALL_OBSERVER_DT_S,
            -BALL_OBSERVER_POSITION_LIMIT_MM,
            BALL_OBSERVER_POSITION_LIMIT_MM);
    }

    if (new_frame != 0U)
    {
        if (observer->initialized == 0U)
        {
            observer->position_mm = clamp_float(
                measurement->position_mm,
                -BALL_OBSERVER_POSITION_LIMIT_MM,
                BALL_OBSERVER_POSITION_LIMIT_MM);
            observer->velocity_mm_s = clamp_float(
                measurement->velocity_mm_s,
                -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
                BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
            observer->acceleration_mm_s2 = 0.0f;
            observer->previous_velocity_mm_s =
                observer->velocity_mm_s;
            observer->initialized = 1U;
            initialized_this_tick = 1U;
        }
        else
        {
            frame_dt_ms =
                measurement->sample_ms -
                observer->last_measurement_ms;
            if (frame_dt_ms < BALL_OBSERVER_MIN_FRAME_DT_MS)
            {
                frame_dt_ms = BALL_OBSERVER_MIN_FRAME_DT_MS;
            }
            else if (frame_dt_ms >
                     BALL_OBSERVER_MAX_FRAME_DT_MS)
            {
                frame_dt_ms = BALL_OBSERVER_MAX_FRAME_DT_MS;
            }

            residual_mm =
                measurement->position_mm -
                observer->position_mm;
            observer->position_mm = clamp_float(
                observer->position_mm +
                    BALL_OBSERVER_ALPHA * residual_mm,
                -BALL_OBSERVER_POSITION_LIMIT_MM,
                BALL_OBSERVER_POSITION_LIMIT_MM);
            observer->velocity_mm_s = clamp_float(
                observer->velocity_mm_s +
                    BALL_OBSERVER_BETA * residual_mm *
                        1000.0f / (float)frame_dt_ms,
                -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
                BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
        }

        observer->last_frame_count = measurement->frame_count;
        observer->last_measurement_ms = measurement->sample_ms;
        observer->valid = 1U;
    }
    else if (measurement_fresh == 0U)
    {
        if (measurement->valid != 0U)
        {
            observer->last_frame_count = measurement->frame_count;
            observer->last_measurement_ms = measurement->sample_ms;
        }
        invalidate_dynamic_state(observer);
    }
    else if (observer->initialized != 0U)
    {
        observer->valid = 1U;
    }

    if (observer->initialized != 0U &&
        initialized_this_tick == 0U)
    {
        acceleration_raw_mm_s2 = clamp_float(
            (observer->velocity_mm_s -
             observer->previous_velocity_mm_s) /
                BALL_OBSERVER_DT_S,
            -BALL_OBSERVER_RAW_ACCEL_LIMIT_MM_S2,
            BALL_OBSERVER_RAW_ACCEL_LIMIT_MM_S2);
        observer->acceleration_mm_s2 = clamp_float(
            observer->acceleration_mm_s2 +
                BALL_OBSERVER_ACCEL_FILTER_ALPHA *
                    (acceleration_raw_mm_s2 -
                     observer->acceleration_mm_s2),
            -BALL_OBSERVER_ACCEL_LIMIT_MM_S2,
            BALL_OBSERVER_ACCEL_LIMIT_MM_S2);
        observer->previous_velocity_mm_s =
            observer->velocity_mm_s;
    }

    observer->update_count++;
}

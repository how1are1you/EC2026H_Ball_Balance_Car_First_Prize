#ifndef BALL_STATE_OBSERVER_H
#define BALL_STATE_OBSERVER_H

#include <stdint.h>

#define BALL_OBSERVER_DT_S                  (0.005f)
#define BALL_OBSERVER_ALPHA                 (0.55f)
#define BALL_OBSERVER_BETA                  (0.10f)
#define BALL_OBSERVER_ACCEL_FILTER_ALPHA    (0.20f)
#define BALL_OBSERVER_POSITION_LIMIT_MM     (150.0f)
#define BALL_OBSERVER_VELOCITY_LIMIT_MM_S   (500.0f)
#define BALL_OBSERVER_RAW_ACCEL_LIMIT_MM_S2 (3000.0f)
#define BALL_OBSERVER_ACCEL_LIMIT_MM_S2     (2000.0f)
#define BALL_OBSERVER_VISION_TIMEOUT_MS     (200UL)
#define BALL_OBSERVER_MIN_FRAME_DT_MS       (5UL)
#define BALL_OBSERVER_MAX_FRAME_DT_MS       (200UL)

typedef struct
{
    float position_mm;
    float velocity_mm_s;
    uint32_t frame_count;
    uint32_t sample_ms;
    uint8_t valid;
} ball_vision_measurement_t;

typedef struct
{
    volatile float position_mm;
    volatile float velocity_mm_s;
    volatile float acceleration_mm_s2;
    volatile uint32_t last_frame_count;
    volatile uint32_t last_measurement_ms;
    volatile uint32_t update_count;
    volatile uint8_t initialized;
    volatile uint8_t valid;
    float previous_velocity_mm_s;
} ball_state_observer_t;

extern ball_state_observer_t ball_state_observer;

void ball_state_observer_reset(ball_state_observer_t *observer);
void ball_state_observer_update(
    ball_state_observer_t *observer,
    const ball_vision_measurement_t *measurement,
    uint32_t now_ms);

#endif

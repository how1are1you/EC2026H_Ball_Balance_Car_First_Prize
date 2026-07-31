#ifndef BALL_POSITION_CAPTURE_H
#define BALL_POSITION_CAPTURE_H

#include <stdint.h>

#define BALL_POSITION_CAPTURE_DURATION_MS (300UL)
#define BALL_POSITION_CAPTURE_MIN_FRAMES (6U)
#define BALL_POSITION_CAPTURE_MAX_SPAN_MM (5.0f)
#define BALL_POSITION_CAPTURE_LIMIT_MM (110.0f)

typedef enum
{
    BALL_POSITION_CAPTURE_WAITING = 0,
    BALL_POSITION_CAPTURE_LOCKED
} ball_position_capture_result_t;

typedef struct
{
    uint32_t start_ms;
    uint32_t last_frame_count;
    uint32_t elapsed_ms;
    uint16_t sample_count;
    uint8_t active;
    float sum_mm;
    float minimum_mm;
    float maximum_mm;
    float mean_mm;
    float span_mm;
} ball_position_capture_t;

void ball_position_capture_reset(
    ball_position_capture_t *capture);
ball_position_capture_result_t ball_position_capture_push(
    ball_position_capture_t *capture,
    float position_mm,
    uint32_t sample_ms,
    uint32_t frame_count,
    float *target_mm);

#endif

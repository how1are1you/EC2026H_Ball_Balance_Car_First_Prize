#include "ball_position_capture.h"

#include <stddef.h>

static void ball_position_capture_start_window(
    ball_position_capture_t *capture,
    float position_mm,
    uint32_t sample_ms,
    uint32_t frame_count)
{
    capture->start_ms = sample_ms;
    capture->last_frame_count = frame_count;
    capture->elapsed_ms = 0U;
    capture->sample_count = 1U;
    capture->active = 1U;
    capture->sum_mm = position_mm;
    capture->minimum_mm = position_mm;
    capture->maximum_mm = position_mm;
    capture->mean_mm = position_mm;
    capture->span_mm = 0.0f;
}

void ball_position_capture_reset(
    ball_position_capture_t *capture)
{
    if (capture == NULL)
    {
        return;
    }

    capture->start_ms = 0U;
    capture->last_frame_count = 0U;
    capture->elapsed_ms = 0U;
    capture->sample_count = 0U;
    capture->active = 0U;
    capture->sum_mm = 0.0f;
    capture->minimum_mm = 0.0f;
    capture->maximum_mm = 0.0f;
    capture->mean_mm = 0.0f;
    capture->span_mm = 0.0f;
}

ball_position_capture_result_t ball_position_capture_push(
    ball_position_capture_t *capture,
    float position_mm,
    uint32_t sample_ms,
    uint32_t frame_count,
    float *target_mm)
{
    float minimum_mm;
    float maximum_mm;

    if (capture == NULL || target_mm == NULL)
    {
        return BALL_POSITION_CAPTURE_WAITING;
    }
    if (capture->last_frame_count == frame_count)
    {
        return BALL_POSITION_CAPTURE_WAITING;
    }
    if (position_mm < -BALL_POSITION_CAPTURE_LIMIT_MM ||
        position_mm > BALL_POSITION_CAPTURE_LIMIT_MM)
    {
        ball_position_capture_reset(capture);
        capture->last_frame_count = frame_count;
        return BALL_POSITION_CAPTURE_WAITING;
    }
    if (capture->active == 0U)
    {
        ball_position_capture_start_window(
            capture,
            position_mm,
            sample_ms,
            frame_count);
        return BALL_POSITION_CAPTURE_WAITING;
    }

    minimum_mm = capture->minimum_mm;
    maximum_mm = capture->maximum_mm;
    if (position_mm < minimum_mm)
    {
        minimum_mm = position_mm;
    }
    if (position_mm > maximum_mm)
    {
        maximum_mm = position_mm;
    }
    if (maximum_mm - minimum_mm >
        BALL_POSITION_CAPTURE_MAX_SPAN_MM)
    {
        ball_position_capture_start_window(
            capture,
            position_mm,
            sample_ms,
            frame_count);
        return BALL_POSITION_CAPTURE_WAITING;
    }

    capture->last_frame_count = frame_count;
    capture->elapsed_ms = sample_ms - capture->start_ms;
    capture->sample_count++;
    capture->sum_mm += position_mm;
    capture->minimum_mm = minimum_mm;
    capture->maximum_mm = maximum_mm;
    capture->mean_mm =
        capture->sum_mm / (float)capture->sample_count;
    capture->span_mm = maximum_mm - minimum_mm;

    if (capture->elapsed_ms >=
            BALL_POSITION_CAPTURE_DURATION_MS &&
        capture->sample_count >=
            BALL_POSITION_CAPTURE_MIN_FRAMES)
    {
        *target_mm = capture->mean_mm;
        return BALL_POSITION_CAPTURE_LOCKED;
    }

    return BALL_POSITION_CAPTURE_WAITING;
}

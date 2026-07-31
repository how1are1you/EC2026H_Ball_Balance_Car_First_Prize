#include "ball_position_capture.h"

#include <assert.h>
#include <math.h>

static void stable_window_locks_average_position(void)
{
    ball_position_capture_t capture;
    float target_mm = 0.0f;
    const float samples_mm[6] =
    {
        50.0f, 51.0f, 49.0f, 50.0f, 50.5f, 49.5f
    };
    unsigned int index;

    ball_position_capture_reset(&capture);
    for (index = 0U; index < 6U; index++)
    {
        ball_position_capture_result_t result =
            ball_position_capture_push(
                &capture,
                samples_mm[index],
                index * 60U,
                index + 1U,
                &target_mm);

        assert(result ==
               ((index == 5U) ?
                    BALL_POSITION_CAPTURE_LOCKED :
                    BALL_POSITION_CAPTURE_WAITING));
    }

    assert(fabsf(target_mm - 50.0f) < 0.01f);
    assert(capture.sample_count == 6U);
    assert(fabsf(capture.span_mm - 2.0f) < 0.01f);
}

static void duplicate_frame_does_not_change_window(void)
{
    ball_position_capture_t capture;
    float target_mm = 0.0f;

    ball_position_capture_reset(&capture);
    (void)ball_position_capture_push(
        &capture, 10.0f, 0U, 1U, &target_mm);
    (void)ball_position_capture_push(
        &capture, 100.0f, 100U, 1U, &target_mm);

    assert(capture.sample_count == 1U);
    assert(fabsf(capture.mean_mm - 10.0f) < 0.01f);
    assert(capture.span_mm == 0.0f);
}

static void excessive_span_restarts_from_latest_sample(void)
{
    ball_position_capture_t capture;
    float target_mm = 0.0f;

    ball_position_capture_reset(&capture);
    (void)ball_position_capture_push(
        &capture, 0.0f, 0U, 1U, &target_mm);
    (void)ball_position_capture_push(
        &capture, 8.0f, 60U, 2U, &target_mm);

    assert(capture.sample_count == 1U);
    assert(fabsf(capture.mean_mm - 8.0f) < 0.01f);
    assert(capture.elapsed_ms == 0U);
}

static void out_of_range_sample_clears_window(void)
{
    ball_position_capture_t capture;
    float target_mm = 0.0f;

    ball_position_capture_reset(&capture);
    (void)ball_position_capture_push(
        &capture, 10.0f, 0U, 1U, &target_mm);
    (void)ball_position_capture_push(
        &capture, 111.0f, 60U, 2U, &target_mm);

    assert(capture.sample_count == 0U);
    assert(capture.active == 0U);
}

int main(void)
{
    stable_window_locks_average_position();
    duplicate_frame_does_not_change_window();
    excessive_span_restarts_from_latest_sample();
    out_of_range_sample_clears_window();
    return 0;
}

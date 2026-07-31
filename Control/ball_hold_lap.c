#include "ball_hold_lap.h"

#include "ball_balance.h"
#include "ball_position_capture.h"
#include "control.h"
#include "straight_turn_test.h"
#include "uart_callback.h"

extern volatile unsigned long tick_ms;

volatile ball_hold_lap_state_t ball_hold_lap_state =
    BALL_HOLD_LAP_READY;
volatile float ball_hold_lap_target_mm;
volatile float ball_hold_lap_current_mm;
volatile float ball_hold_lap_error_mm;
volatile float ball_hold_lap_max_abs_error_mm;
volatile float ball_hold_lap_capture_mean_mm;
volatile float ball_hold_lap_capture_span_mm;
volatile uint16_t ball_hold_lap_capture_frames;
volatile uint32_t ball_hold_lap_capture_elapsed_ms;
volatile uint32_t ball_hold_lap_time_ms;
volatile uint8_t ball_hold_lap_current_valid;
volatile uint8_t ball_hold_lap_time_pass;
volatile uint8_t ball_hold_lap_position_pass;
volatile uint8_t ball_hold_lap_fault;

static ball_position_capture_t ball_hold_capture;
static uint32_t ball_hold_last_measurement_frame;

static uint8_t ball_hold_lap_read_vision(
    float *position_mm,
    uint32_t *frame_count,
    uint32_t *sample_ms)
{
    uint32_t frame_before;
    uint32_t frame_after;

    if (vision_ball_position_valid == 0U)
    {
        return 0U;
    }
    if ((uint32_t)tick_ms - vision_ball_last_update_ms >
        BALL_HOLD_LAP_VISION_TIMEOUT_MS)
    {
        return 0U;
    }

    do
    {
        frame_before = vision_ball_frame_count;
        *position_mm = vision_ball_position_mm;
        *sample_ms = vision_ball_last_update_ms;
        frame_after = vision_ball_frame_count;
    } while (frame_before != frame_after);

    *frame_count = frame_after;
    return 1U;
}

static void ball_hold_lap_publish_capture(void)
{
    ball_hold_lap_capture_frames =
        ball_hold_capture.sample_count;
    ball_hold_lap_capture_elapsed_ms =
        ball_hold_capture.elapsed_ms;
    ball_hold_lap_capture_mean_mm =
        ball_hold_capture.mean_mm;
    ball_hold_lap_capture_span_mm =
        ball_hold_capture.span_mm;
}

static void ball_hold_lap_clear_results(void)
{
    ball_hold_lap_target_mm = 0.0f;
    ball_hold_lap_current_mm = 0.0f;
    ball_hold_lap_error_mm = 0.0f;
    ball_hold_lap_max_abs_error_mm = 0.0f;
    ball_hold_lap_capture_mean_mm = 0.0f;
    ball_hold_lap_capture_span_mm = 0.0f;
    ball_hold_lap_capture_frames = 0U;
    ball_hold_lap_capture_elapsed_ms = 0U;
    ball_hold_lap_time_ms = 0U;
    ball_hold_lap_current_valid = 0U;
    ball_hold_lap_time_pass = 1U;
    ball_hold_lap_position_pass = 1U;
    ball_hold_lap_fault = 0U;
    ball_hold_last_measurement_frame = 0U;
}

void ball_hold_lap_reset(void)
{
    StraightTurnTest_Reset();
    ball_position_capture_reset(&ball_hold_capture);
    ball_hold_lap_clear_results();
    ball_hold_lap_state = BALL_HOLD_LAP_READY;
}

void ball_hold_lap_init(void)
{
    ball_hold_lap_reset();
}

void ball_hold_lap_start(void)
{
    StraightTurnTest_Reset();
    ball_position_capture_reset(&ball_hold_capture);
    ball_hold_lap_clear_results();
    Flag_Stop = 1U;
    ball_hold_lap_state = BALL_HOLD_LAP_CAPTURING;
}

void ball_hold_lap_stop(void)
{
    uint8_t was_active =
        (ball_hold_lap_state == BALL_HOLD_LAP_CAPTURING ||
         ball_hold_lap_state == BALL_HOLD_LAP_RUNNING ||
         ball_hold_lap_state == BALL_HOLD_LAP_POST_LAP ||
         ball_hold_lap_state == BALL_HOLD_LAP_BRAKING);

    StraightTurnTest_Stop();
    ball_position_capture_reset(&ball_hold_capture);
    if (was_active != 0U)
    {
        ball_hold_lap_state = BALL_HOLD_LAP_ABORTED;
    }
}

static void ball_hold_lap_update_capture(void)
{
    float position_mm;
    float target_mm;
    uint32_t frame_count;
    uint32_t sample_ms;
    ball_position_capture_result_t result;

    if (ball_hold_lap_read_vision(
            &position_mm,
            &frame_count,
            &sample_ms) == 0U)
    {
        ball_hold_lap_current_valid = 0U;
        ball_position_capture_reset(&ball_hold_capture);
        ball_hold_lap_publish_capture();
        return;
    }

    ball_hold_lap_current_valid = 1U;
    ball_hold_lap_current_mm = position_mm;
    result = ball_position_capture_push(
        &ball_hold_capture,
        position_mm,
        sample_ms,
        frame_count,
        &target_mm);
    ball_hold_lap_publish_capture();
    if (result != BALL_POSITION_CAPTURE_LOCKED)
    {
        return;
    }

    ball_hold_lap_target_mm = target_mm;
    ball_hold_lap_current_mm = position_mm;
    ball_hold_last_measurement_frame = frame_count;
    ball_balance_set_reference(target_mm, 0.0f);
    StraightTurnTest_StartWithPostLap(
        STRAIGHT_TURN_BALL_SPEED_MPS,
        STRAIGHT_TURN_BALL_ACCELERATION_MPS2,
        BALL_HOLD_LAP_POST_DISTANCE_M);
    if (StraightTurnState == STRAIGHT_TURN_FAULT)
    {
        ball_hold_lap_fault = StraightTurnFault;
        ball_hold_lap_state = BALL_HOLD_LAP_FAULT;
    }
    else
    {
        ball_hold_lap_state = BALL_HOLD_LAP_RUNNING;
    }
}

static void ball_hold_lap_update_measurement(void)
{
    float position_mm;
    float error_mm;
    float absolute_error_mm;
    uint32_t frame_count;
    uint32_t sample_ms;

    if (ball_hold_lap_read_vision(
            &position_mm,
            &frame_count,
            &sample_ms) == 0U)
    {
        ball_hold_lap_current_valid = 0U;
        return;
    }
    (void)sample_ms;

    ball_hold_lap_current_valid = 1U;
    ball_hold_lap_current_mm = position_mm;
    if (frame_count == ball_hold_last_measurement_frame)
    {
        return;
    }
    ball_hold_last_measurement_frame = frame_count;

    error_mm = position_mm - ball_hold_lap_target_mm;
    absolute_error_mm =
        (error_mm < 0.0f) ? -error_mm : error_mm;
    ball_hold_lap_error_mm = error_mm;
    if (absolute_error_mm >
        ball_hold_lap_max_abs_error_mm)
    {
        ball_hold_lap_max_abs_error_mm =
            absolute_error_mm;
    }
    if (absolute_error_mm >
        BALL_HOLD_LAP_MAX_ERROR_MM)
    {
        ball_hold_lap_position_pass = 0U;
    }
}

static void ball_hold_lap_update_course_state(void)
{
    if (StraightTurnLapTimeMs != 0U &&
        ball_hold_lap_time_ms == 0U)
    {
        ball_hold_lap_time_ms =
            StraightTurnLapTimeMs;
        ball_hold_lap_time_pass =
            (StraightTurnLapTimeMs <=
             BALL_HOLD_LAP_TIME_LIMIT_MS);
    }

    if (StraightTurnState == STRAIGHT_TURN_POST_LAP)
    {
        ball_hold_lap_state = BALL_HOLD_LAP_POST_LAP;
    }
    else if (StraightTurnState == STRAIGHT_TURN_BRAKING)
    {
        ball_hold_lap_state = BALL_HOLD_LAP_BRAKING;
    }
    else if (StraightTurnState == STRAIGHT_TURN_DONE)
    {
        ball_hold_lap_state = BALL_HOLD_LAP_DONE;
    }
    else if (StraightTurnState == STRAIGHT_TURN_FAULT)
    {
        ball_hold_lap_fault = StraightTurnFault;
        ball_hold_lap_state = BALL_HOLD_LAP_FAULT;
    }
    else
    {
        ball_hold_lap_state = BALL_HOLD_LAP_RUNNING;
    }
}

void ball_hold_lap_update(void)
{
    if (ball_hold_lap_state == BALL_HOLD_LAP_CAPTURING)
    {
        ball_hold_lap_update_capture();
        return;
    }
    if (ball_hold_lap_state != BALL_HOLD_LAP_RUNNING &&
        ball_hold_lap_state != BALL_HOLD_LAP_POST_LAP &&
        ball_hold_lap_state != BALL_HOLD_LAP_BRAKING)
    {
        return;
    }

    ball_hold_lap_update_measurement();
    ball_hold_lap_update_course_state();
}

uint8_t ball_hold_lap_controller_enabled(void)
{
    return
        (ball_hold_lap_state == BALL_HOLD_LAP_RUNNING ||
         ball_hold_lap_state == BALL_HOLD_LAP_POST_LAP ||
         ball_hold_lap_state == BALL_HOLD_LAP_BRAKING) ?
            1U : 0U;
}

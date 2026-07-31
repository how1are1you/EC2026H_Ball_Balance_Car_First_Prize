#include <assert.h>
#include <math.h>
#include <stdint.h>

#define __CONTROL_H

unsigned char Flag_Stop = 1U;

#include "../Control/ball_hold_lap.c"

volatile unsigned long tick_ms;

volatile float vision_ball_position_mm;
volatile float vision_ball_velocity_mm_s;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint32_t vision_uart_error_count;
volatile uint8_t vision_ball_position_valid;

volatile StraightTurnState_t StraightTurnState =
    STRAIGHT_TURN_IDLE;
volatile uint8_t StraightTurnFault;
volatile float StraightTurnDistanceM;
volatile float StraightTurnYawDeg;
volatile float StraightTurnLineError;
volatile float StraightTurnHeadingErrorDeg;
volatile float StraightTurnCommandSpeed;
volatile float StraightTurnStartupAccelerationMps2;
volatile uint32_t StraightTurnElapsedMs;
volatile uint32_t StraightTurnLapTimeMs;
volatile float StraightTurnPostLapDistanceM;

volatile float ball_balance_target_mm;

static unsigned int straight_turn_start_count;
static float started_post_lap_distance_m;

void ball_balance_set_reference(
    float position_mm,
    float velocity_mm_s)
{
    (void)velocity_mm_s;
    ball_balance_target_mm = position_mm;
}

void StraightTurnTest_Reset(void)
{
    StraightTurnState = STRAIGHT_TURN_IDLE;
    StraightTurnFault = 0U;
    StraightTurnElapsedMs = 0U;
    StraightTurnLapTimeMs = 0U;
    StraightTurnPostLapDistanceM = 0.0f;
    Flag_Stop = 1U;
}

void StraightTurnTest_StartWithPostLap(
    float target_speed_mps,
    float acceleration_mps2,
    float post_lap_distance_m)
{
    (void)target_speed_mps;
    (void)acceleration_mps2;
    straight_turn_start_count++;
    started_post_lap_distance_m = post_lap_distance_m;
    StraightTurnState = STRAIGHT_TURN_STRAIGHT_1;
    Flag_Stop = 0U;
}

void StraightTurnTest_Stop(void)
{
    StraightTurnState = STRAIGHT_TURN_IDLE;
    Flag_Stop = 1U;
}

static void reset_test(void)
{
    tick_ms = 0UL;
    vision_ball_position_mm = 0.0f;
    vision_ball_velocity_mm_s = 0.0f;
    vision_ball_frame_count = 0U;
    vision_ball_last_update_ms = 0U;
    vision_ball_position_valid = 0U;
    straight_turn_start_count = 0U;
    started_post_lap_distance_m = 0.0f;
    ball_balance_target_mm = 0.0f;
    ball_hold_lap_init();
}

static void publish_position(
    float position_mm,
    uint32_t sample_ms)
{
    tick_ms = sample_ms;
    vision_ball_position_mm = position_mm;
    vision_ball_last_update_ms = sample_ms;
    vision_ball_frame_count++;
    vision_ball_position_valid = 1U;
}

static void stable_capture_starts_lap_at_average_position(void)
{
    const float samples_mm[6] =
    {
        50.0f, 51.0f, 49.0f, 50.0f, 50.5f, 49.5f
    };
    unsigned int index;

    reset_test();
    ball_hold_lap_start();
    for (index = 0U; index < 6U; index++)
    {
        publish_position(samples_mm[index], index * 60U);
        ball_hold_lap_update();
    }

    assert(straight_turn_start_count == 1U);
    assert(fabsf(started_post_lap_distance_m - 1.0f) < 0.001f);
    assert(fabsf(ball_hold_lap_target_mm - 50.0f) < 0.01f);
    assert(fabsf(ball_balance_target_mm - 50.0f) < 0.01f);
    assert(ball_hold_lap_state == BALL_HOLD_LAP_RUNNING);
    assert(ball_hold_lap_controller_enabled() == 1U);
}

static void running_task_latches_error_and_lap_result(void)
{
    unsigned int index;

    reset_test();
    ball_hold_lap_start();
    for (index = 0U; index < 6U; index++)
    {
        publish_position(40.0f, index * 60U);
        ball_hold_lap_update();
    }

    publish_position(52.0f, 360U);
    StraightTurnLapTimeMs = 25000U;
    StraightTurnPostLapDistanceM = 0.25f;
    StraightTurnState = STRAIGHT_TURN_POST_LAP;
    ball_hold_lap_update();

    assert(fabsf(ball_hold_lap_error_mm - 12.0f) < 0.01f);
    assert(fabsf(ball_hold_lap_max_abs_error_mm - 12.0f) < 0.01f);
    assert(ball_hold_lap_position_pass == 0U);
    assert(ball_hold_lap_time_ms == 25000U);
    assert(ball_hold_lap_time_pass == 1U);
    assert(ball_hold_lap_state == BALL_HOLD_LAP_POST_LAP);
}

static void stale_vision_does_not_abort_running_task(void)
{
    unsigned int index;

    reset_test();
    ball_hold_lap_start();
    for (index = 0U; index < 6U; index++)
    {
        publish_position(0.0f, index * 60U);
        ball_hold_lap_update();
    }

    tick_ms = 600UL;
    ball_hold_lap_update();

    assert(ball_hold_lap_state == BALL_HOLD_LAP_RUNNING);
    assert(ball_hold_lap_current_valid == 0U);
    assert(ball_hold_lap_controller_enabled() == 1U);
}

int main(void)
{
    stable_capture_starts_lap_at_average_position();
    running_task_latches_error_and_lap_result();
    stale_vision_does_not_abort_running_task();
    return 0;
}

#include <assert.h>
#include <math.h>
#include <stdint.h>

volatile unsigned long tick_ms;

volatile float vision_ball_position_mm;
volatile float vision_ball_velocity_mm_s;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint32_t vision_uart_error_count;
volatile uint8_t vision_ball_position_valid;

volatile float ball_balance_target_mm;
volatile float ball_balance_reference_velocity_mm_s;

static uint16_t test_servo_pulse_us;

void ball_balance_set_reference(
    float position_mm,
    float velocity_mm_s)
{
    ball_balance_target_mm = position_mm;
    ball_balance_reference_velocity_mm_s = velocity_mm_s;
}

void servo_set_pulse_us(uint16_t pulse_us)
{
    test_servo_pulse_us = pulse_us;
}

uint16_t servo_get_pulse_us(void)
{
    return test_servo_pulse_us;
}

#include "../Control/ball_static_task.c"

static void publish_sample(
    float position_mm,
    float velocity_mm_s,
    uint32_t sample_ms)
{
    tick_ms = sample_ms;
    vision_ball_position_mm = position_mm;
    vision_ball_velocity_mm_s = velocity_mm_s;
    vision_ball_last_update_ms = sample_ms;
    vision_ball_frame_count++;
    vision_ball_position_valid = 1U;
}

static void ready_and_start(void)
{
    tick_ms = 0UL;
    vision_ball_position_mm = 0.0f;
    vision_ball_velocity_mm_s = 0.0f;
    vision_ball_frame_count = 0U;
    vision_ball_last_update_ms = 0U;
    vision_ball_position_valid = 0U;
    ball_balance_target_mm = 0.0f;
    ball_balance_reference_velocity_mm_s = 0.0f;
    test_servo_pulse_us = 0U;

    ball_static_task_init();
    publish_sample(0.0f, 0.0f, 1U);
    ball_static_task_update();
    publish_sample(0.0f, 0.0f, 201U);
    ball_static_task_update();
    assert(ball_static_task_start() == 1U);
}

static void positive_settle_then_reverse(void)
{
    ready_and_start();

    assert(ball_static_task_controller_enabled() == 1U);
    assert(fabsf(ball_balance_target_mm - 50.0f) < 0.01f);

    publish_sample(50.0f, 4.0f, 500U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_HOLD_POS);

    publish_sample(50.0f, 3.0f, 599U);
    ball_static_task_update();
    assert(fabsf(ball_static_target_mm - 50.0f) < 0.01f);

    publish_sample(50.0f, 3.0f, 600U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_MOVE_NEG);
    assert(fabsf(ball_static_target_mm + 55.0f) < 0.01f);
    assert(fabsf(ball_static_positive_max_error_mm) < 0.01f);
}

static void negative_end_completes_only_after_300ms_settle(void)
{
    positive_settle_then_reverse();

    publish_sample(-55.0f, 4.0f, 900U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_HOLD_NEG);

    publish_sample(-55.0f, 3.0f, 1199U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_HOLD_NEG);

    publish_sample(-55.0f, 3.0f, 1200U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_DONE);
    assert(ball_static_task_controller_enabled() == 1U);
    assert(fabsf(ball_balance_target_mm + 55.0f) < 0.01f);
}

static void task_keeps_positive_pid_enabled_after_five_seconds(void)
{
    ready_and_start();

    publish_sample(20.0f, 20.0f, 5202U);
    ball_static_task_update();

    assert(ball_static_state == BALL_STATIC_MOVE_POS);
    assert(ball_static_task_controller_enabled() == 1U);
    assert(fabsf(ball_balance_target_mm - 50.0f) < 0.01f);
}

int main(void)
{
    positive_settle_then_reverse();
    negative_end_completes_only_after_300ms_settle();
    task_keeps_positive_pid_enabled_after_five_seconds();
    return 0;
}

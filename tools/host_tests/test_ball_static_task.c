#include "ball_balance.h"
#include "ball_state_observer.h"
#include "ball_static_task.h"
#include "servo.h"

#include <assert.h>

volatile unsigned long tick_ms;
ball_state_observer_t ball_state_observer;

static uint16_t fake_servo_pulse_us;
static float fake_reference_position_mm;
static float fake_reference_velocity_mm_s;

void servo_init(void)
{
}

void servo_set_pulse_us(uint16_t pulse_us)
{
    fake_servo_pulse_us = pulse_us;
}

uint16_t servo_get_pulse_us(void)
{
    return fake_servo_pulse_us;
}

void ball_balance_set_reference(
    float position_mm,
    float velocity_mm_s)
{
    fake_reference_position_mm = position_mm;
    fake_reference_velocity_mm_s = velocity_mm_s;
}

static void step_observer(
    float position_mm,
    float velocity_mm_s)
{
    tick_ms += 5U;
    ball_state_observer.position_mm = position_mm;
    ball_state_observer.velocity_mm_s = velocity_mm_s;
    ball_state_observer.valid = 1U;
    ball_static_task_update();
}

static void hold_observer(
    float position_mm,
    float velocity_mm_s,
    uint32_t duration_ms)
{
    uint32_t elapsed_ms;

    for (elapsed_ms = 0U;
         elapsed_ms < duration_ms;
         elapsed_ms += 5U)
    {
        step_observer(position_mm, velocity_mm_s);
    }
}

int main(void)
{
    tick_ms = 0U;
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_state_observer.valid = 1U;
    ball_static_task_init();

    hold_observer(0.0f, 0.0f, 210U);
    assert(ball_static_ready == 1U);
    assert(ball_static_task_start() == 1U);
    assert(ball_static_state == BALL_STATIC_MOVE_POS);

    step_observer(40.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_MOVE_POS);

    step_observer(50.0f, 11.0f);
    assert(ball_static_state == BALL_STATIC_MOVE_POS);

    step_observer(50.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_HOLD_POS);

    step_observer(44.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_MOVE_POS);

    step_observer(50.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_HOLD_POS);
    hold_observer(50.0f, 0.0f, 95U);
    assert(ball_static_state == BALL_STATIC_HOLD_POS);
    step_observer(50.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_MOVE_NEG);
    assert(fake_reference_position_mm == -50.0f);
    assert(fake_reference_velocity_mm_s == 0.0f);

    step_observer(-50.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_HOLD_NEG);
    hold_observer(-50.0f, 0.0f, 295U);
    assert(ball_static_state == BALL_STATIC_HOLD_NEG);
    step_observer(-50.0f, 0.0f);
    assert(ball_static_state == BALL_STATIC_DONE);
    assert(fake_servo_pulse_us >= SERVO_CONTROL_MIN_PULSE_US);
    assert(fake_servo_pulse_us <= SERVO_CONTROL_MAX_PULSE_US);
    return 0;
}

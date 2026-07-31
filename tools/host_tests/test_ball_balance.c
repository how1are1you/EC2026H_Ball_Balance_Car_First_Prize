#include "ball_balance.h"
#include "ball_state_observer.h"
#include "servo.h"

#include <assert.h>
#include <math.h>

volatile unsigned long tick_ms;
volatile float vision_ball_position_mm;
volatile float vision_ball_velocity_mm_s;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint8_t vision_ball_position_valid;

static uint16_t fake_servo_pulse_us;

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

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    ball_state_observer_reset(&ball_state_observer);
    ball_state_observer.valid = 1U;
    ball_state_observer.initialized = 1U;
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 10.0f;
    ball_state_observer.acceleration_mm_s2 = 20.0f;

    ball_balance_init();
    assert(ball_balance_set_cascade_gains(
               0.0f, 0.0f, 1.0f, 1.0f, 500.0f) == 1U);
    ball_balance_set_enabled(1U);
    ball_balance_set_reference(0.0f, 0.0f);
    ball_balance_update();

    assert(ball_balance_update_count == 1U);
    assert_close(ball_balance_proportional_us, -10.0f);
    assert_close(ball_balance_derivative_us, -20.0f);
    assert(fake_servo_pulse_us == 1220U);

    ball_state_observer.velocity_mm_s = 0.0f;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 2U);
    assert_close(ball_balance_derivative_us, 0.0f);

    ball_state_observer.valid = 0U;
    ball_balance_update();
    assert(ball_balance_update_count == 3U);
    assert(fake_servo_pulse_us == SERVO_NEUTRAL_PULSE_US);

    return 0;
}

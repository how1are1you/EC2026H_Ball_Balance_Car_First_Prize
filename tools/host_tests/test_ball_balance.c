#include "ball_balance.h"
#include "ball_state_observer.h"
#include "servo.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

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

static void assert_close_at(float actual, float expected, int line)
{
    if (fabsf(actual - expected) >= 0.001f)
    {
        fprintf(
            stderr,
            "line %d: expected %.3f, got %.3f\n",
            line,
            expected,
            actual);
        assert(0);
    }
}

#define assert_close(actual, expected) \
    assert_close_at((actual), (expected), __LINE__)

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
    assert_close(ball_balance_position_pid_velocity_mm_s, 0.0f);
    assert_close(ball_balance_distance_velocity_limit_mm_s, 0.0f);
    assert(ball_balance_distance_limited == 0U);
    assert_close(ball_balance_proportional_us, -10.0f);
    assert_close(ball_balance_derivative_us, -20.0f);
    assert_close(ball_balance_unsaturated_pulse_us, 1345.0f);
    assert(ball_balance_output_saturated == 0U);
    assert(fake_servo_pulse_us == 1345U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 0.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_state_observer.position_mm = 20.0f;
    ball_state_observer.velocity_mm_s = 60.0f;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 2U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 60.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        40.66457f);
    assert_close(ball_balance_target_velocity_mm_s, 40.66457f);
    assert(ball_balance_distance_limited == 1U);
    assert_close(ball_balance_proportional_us, -19.33543f);
    assert_close(ball_balance_unsaturated_pulse_us, 1383.6646f);
    assert(fake_servo_pulse_us == 1385U);

    ball_state_observer.position_mm = 30.0f;
    ball_state_observer.velocity_mm_s = 80.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 3U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 40.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        32.88775f);
    assert_close(ball_balance_target_velocity_mm_s, 32.88775f);
    assert(ball_balance_distance_limited == 1U);
    assert_close(ball_balance_proportional_us, -47.11225f);
    assert_close(ball_balance_unsaturated_pulse_us, 1371.8877f);
    assert(fake_servo_pulse_us == 1370U);

    ball_state_observer.position_mm = 35.0f;
    ball_state_observer.velocity_mm_s = 20.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 4U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 30.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        28.25395f);
    assert_close(ball_balance_target_velocity_mm_s, 28.25395f);
    assert(ball_balance_distance_limited == 1U);
    assert_close(ball_balance_proportional_us, 8.25395f);
    assert_close(ball_balance_unsaturated_pulse_us, 1436.254f);
    assert(fake_servo_pulse_us == 1435U);

    ball_state_observer.position_mm = 40.0f;
    ball_state_observer.velocity_mm_s = 5.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 5U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 20.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        22.76094f);
    assert_close(ball_balance_target_velocity_mm_s, 20.0f);
    assert(ball_balance_distance_limited == 0U);

    ball_state_observer.position_mm = 46.0f;
    ball_state_observer.velocity_mm_s = 5.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 6U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 8.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        13.79615f);
    assert_close(ball_balance_target_velocity_mm_s, 8.0f);
    assert(ball_balance_distance_limited == 0U);

    ball_state_observer.position_mm = 30.0f;
    ball_state_observer.velocity_mm_s = 80.0f;
    ball_balance_set_reference(50.0f, 10.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 7U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 50.0f);
    assert_close(ball_balance_distance_velocity_limit_mm_s, 0.0f);
    assert_close(ball_balance_target_velocity_mm_s, 50.0f);
    assert(ball_balance_distance_limited == 0U);

    ball_state_observer.position_mm = -30.0f;
    ball_state_observer.velocity_mm_s = -80.0f;
    ball_balance_set_reference(-50.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 8U);
    assert_close(ball_balance_position_pid_velocity_mm_s, -40.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        32.88775f);
    assert_close(ball_balance_target_velocity_mm_s, -32.88775f);
    assert(ball_balance_distance_limited == 1U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 1.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_state_observer.position_mm = 20.0f;
    ball_state_observer.velocity_mm_s = 40.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
    {
        ball_balance_update();
        assert(ball_balance_distance_limited == 1U);
        assert_close(
            ball_balance_target_velocity_mm_s,
            40.66457f);
    }

    ball_state_observer.position_mm = 46.0f;
    ball_state_observer.velocity_mm_s = 5.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 109U);
    assert(ball_balance_distance_limited == 0U);
    assert_close(ball_balance_target_velocity_mm_s, 8.02f);

    assert(ball_balance_set_cascade_gains(
               0.0f, 0.0f, 20.0f, 0.0f, 500.0f) == 1U);
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = -500.0f;
    ball_balance_set_reference(0.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 110U);
    assert(ball_balance_output_saturated == 1U);
    assert(ball_balance_unsaturated_pulse_us > 2200.0f);
    assert(fake_servo_pulse_us == SERVO_CONTROL_MAX_PULSE_US);

    ball_state_observer.velocity_mm_s = 500.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 111U);
    assert(ball_balance_output_saturated == 1U);
    assert(ball_balance_unsaturated_pulse_us < 500.0f);
    assert(fake_servo_pulse_us == SERVO_CONTROL_MIN_PULSE_US);

    assert(ball_balance_set_cascade_gains(
               0.0f, 1.0f, 20.0f, 0.0f, 500.0f) == 1U);
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = -500.0f;
    ball_balance_set_reference(100.0f, 0.0f);
    for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
    {
        ball_balance_update();
        assert(ball_balance_output_saturated == 1U);
    }

    ball_state_observer.velocity_mm_s = 0.0f;
    ball_balance_update();
    assert(ball_balance_target_velocity_mm_s < 1.0f);
    assert(ball_balance_output_saturated == 0U);

    ball_state_observer.valid = 0U;
    ball_balance_update();
    assert(ball_balance_update_count == 213U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 0.0f);
    assert_close(ball_balance_distance_velocity_limit_mm_s, 0.0f);
    assert(ball_balance_distance_limited == 0U);
    assert(ball_balance_output_saturated == 0U);
    assert(fake_servo_pulse_us == SERVO_NEUTRAL_PULSE_US);
    return 0;
}

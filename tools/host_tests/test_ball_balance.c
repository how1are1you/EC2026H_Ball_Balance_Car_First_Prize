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
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 60.0f;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 2U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 100.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        79.57136f);
    assert_close(ball_balance_target_velocity_mm_s, 79.57136f);
    assert(ball_balance_distance_limited == 1U);
    assert_close(ball_balance_proportional_us, 19.57136f);
    assert_close(ball_balance_unsaturated_pulse_us, 1394.5714f);
    assert(fake_servo_pulse_us == 1395U);

    ball_state_observer.position_mm = 20.0f;
    ball_state_observer.velocity_mm_s = 60.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 3U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 60.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        60.74336f);
    assert_close(ball_balance_target_velocity_mm_s, 60.0f);
    assert(ball_balance_distance_limited == 0U);
    assert_close(ball_balance_proportional_us, 0.0f);
    assert_close(ball_balance_unsaturated_pulse_us, 1403.0f);
    assert(fake_servo_pulse_us == 1405U);

    ball_state_observer.position_mm = 30.0f;
    ball_state_observer.velocity_mm_s = 80.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 4U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 40.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        48.88145f);
    assert_close(ball_balance_target_velocity_mm_s, 40.0f);
    assert(ball_balance_distance_limited == 0U);
    assert_close(ball_balance_proportional_us, -40.0f);
    assert_close(ball_balance_unsaturated_pulse_us, 1379.0f);
    assert(fake_servo_pulse_us == 1380U);

    ball_state_observer.position_mm = 35.0f;
    ball_state_observer.velocity_mm_s = 20.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 5U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 30.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        41.81782f);
    assert_close(ball_balance_target_velocity_mm_s, 30.0f);
    assert(ball_balance_distance_limited == 0U);
    assert_close(ball_balance_proportional_us, 10.0f);
    assert_close(ball_balance_unsaturated_pulse_us, 1438.0f);
    assert(fake_servo_pulse_us == 1440U);

    ball_state_observer.position_mm = 40.0f;
    ball_state_observer.velocity_mm_s = 5.0f;
    ball_balance_update();
    assert(ball_balance_update_count == 6U);
    assert_close(ball_balance_position_pid_velocity_mm_s, 20.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        33.45156f);
    assert_close(ball_balance_target_velocity_mm_s, 20.0f);
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

    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = -60.0f;
    ball_balance_set_reference(-50.0f, 0.0f);
    ball_balance_update();
    assert(ball_balance_update_count == 8U);
    assert_close(ball_balance_position_pid_velocity_mm_s, -100.0f);
    assert_close(
        ball_balance_distance_velocity_limit_mm_s,
        79.57136f);
    assert_close(ball_balance_target_velocity_mm_s, -79.57136f);
    assert(ball_balance_distance_limited == 1U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 1.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 40.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
    {
        ball_balance_update();
        assert(ball_balance_distance_limited == 1U);
        assert_close(
            ball_balance_target_velocity_mm_s,
            79.57136f);
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

    ball_state_observer.valid = 1U;
    ball_state_observer.initialized = 1U;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    assert(ball_balance_set_cascade_gains(
               2.0f, 0.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_balance_set_reference(0.0f, 0.0f);
    ball_balance_set_predictive_guard_enabled(1U);

    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = 50.0f;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, -9.0f);
    assert_close(ball_balance_guard_velocity_mm_s, -32.0f);
    assert_close(ball_balance_position_pid_velocity_mm_s, -12.0f);
    assert_close(ball_balance_target_velocity_mm_s, -32.0f);
    assert(ball_balance_guard_active == 1U);

    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = -50.0f;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, -3.0f);
    assert_close(ball_balance_guard_velocity_mm_s, 0.0f);
    assert_close(ball_balance_target_velocity_mm_s, -12.0f);
    assert(ball_balance_guard_active == 0U);

    ball_state_observer.position_mm = 9.0f;
    ball_state_observer.velocity_mm_s = -50.0f;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, -6.0f);
    assert_close(ball_balance_guard_velocity_mm_s, -8.0f);
    assert_close(ball_balance_target_velocity_mm_s, -18.0f);
    assert(ball_balance_guard_active == 0U);

    ball_state_observer.position_mm = -6.0f;
    ball_state_observer.velocity_mm_s = -50.0f;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, 9.0f);
    assert_close(ball_balance_guard_velocity_mm_s, 32.0f);
    assert_close(ball_balance_target_velocity_mm_s, 32.0f);
    assert(ball_balance_guard_active == 1U);

    ball_balance_set_predictive_guard_enabled(0U);
    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = 50.0f;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, 0.0f);
    assert_close(ball_balance_guard_velocity_mm_s, 0.0f);
    assert_close(ball_balance_target_velocity_mm_s, -12.0f);
    assert(ball_balance_guard_active == 0U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 1.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_balance_set_predictive_guard_enabled(1U);
    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = 50.0f;
    for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
    {
        ball_balance_update();
        assert_close(ball_balance_target_velocity_mm_s, -32.0f);
        assert(ball_balance_guard_active == 1U);
    }

    ball_balance_set_predictive_guard_enabled(0U);
    ball_state_observer.position_mm = 4.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_balance_update();
    assert_close(ball_balance_target_velocity_mm_s, -8.02f);
    assert(ball_balance_guard_active == 0U);

    ball_balance_set_predictive_guard_enabled(1U);
    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = 50.0f;
    ball_balance_update();
    assert(ball_balance_guard_active == 1U);
    ball_state_observer.valid = 0U;
    ball_balance_update();
    assert_close(ball_balance_predicted_error_mm, 0.0f);
    assert_close(ball_balance_guard_velocity_mm_s, 0.0f);
    assert(ball_balance_guard_active == 0U);

    ball_state_observer.valid = 1U;
    ball_state_observer.initialized = 1U;
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    assert(ball_balance_set_cascade_gains(
               0.0f, 0.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_balance_set_predictive_guard_enabled(0U);
    ball_balance_set_reference(0.0f, 0.0f);
    ball_balance_set_turn_feedforward(0.0f);
    ball_balance_update();

    ball_balance_set_turn_feedforward(-50.0f);
    ball_balance_update();
    assert_close(ball_balance_turn_feedforward_us, -50.0f);
    assert(ball_balance_turn_feedforward_active == 1U);
    assert_close(
        ball_balance_unsaturated_pulse_us,
        ball_balance_position_feedforward_us - 50.0f);

    ball_balance_set_turn_feedforward(-200.0f);
    assert_close(ball_balance_turn_feedforward_us, -100.0f);
    ball_balance_set_turn_feedforward(NAN);
    assert_close(ball_balance_turn_feedforward_us, 0.0f);
    assert(ball_balance_turn_feedforward_active == 0U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 1.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_balance_set_turn_feedforward(0.0f);
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_balance_update();

    ball_balance_set_turn_feedforward(-30.0f);
    ball_state_observer.position_mm = 6.0f;
    for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
    {
        ball_balance_update();
        assert(ball_balance_turn_feedforward_active == 1U);
    }

    ball_balance_set_turn_feedforward(0.0f);
    ball_state_observer.position_mm = 4.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_balance_update();
    assert_close(ball_balance_target_velocity_mm_s, -8.02f);
    assert(ball_balance_turn_feedforward_active == 0U);

    assert(ball_balance_set_cascade_gains(
               2.0f, 0.0f, 1.0f, 0.0f, 500.0f) == 1U);
    ball_balance_set_turn_feedforward(0.0f);
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 0.0f;
    ball_balance_update();

    ball_balance_set_predictive_guard_enabled(1U);
    ball_balance_set_turn_feedforward(-30.0f);
    ball_state_observer.position_mm = 6.0f;
    ball_state_observer.velocity_mm_s = 50.0f;
    ball_balance_update();
    assert(ball_balance_guard_active == 1U);
    assert_close(ball_balance_target_velocity_mm_s, -32.0f);
    assert_close(ball_balance_turn_feedforward_us, -30.0f);
    assert_close(
        ball_balance_unsaturated_pulse_us,
        ball_balance_position_feedforward_us +
            ball_balance_proportional_us +
            ball_balance_derivative_us +
            ball_balance_acceleration_feedforward_us -
            30.0f);

    ball_balance_set_enabled(0U);
    assert_close(ball_balance_turn_feedforward_us, 0.0f);
    assert(ball_balance_turn_feedforward_active == 0U);

    return 0;
}

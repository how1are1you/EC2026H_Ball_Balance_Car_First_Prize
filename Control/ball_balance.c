#include "ball_balance.h"

#include "ball_braking.h"
#include "ball_position_feedforward.h"
#include "ball_state_observer.h"
#include "servo.h"

#define BALL_BALANCE_POSITION_DEADBAND_MM (1.0f)
#define BALL_BALANCE_VELOCITY_DEADBAND_MM_S (2.0f)
#define BALL_BALANCE_POSITION_INTEGRAL_LIMIT_MM_S (300.0f)
#define BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S (1.0f)

volatile float ball_balance_target_mm = BALL_BALANCE_TARGET_MM;
volatile float ball_balance_reference_velocity_mm_s;
volatile float ball_balance_position_kp =
    BALL_BALANCE_POSITION_KP_PER_S;
volatile float ball_balance_position_ki =
    BALL_BALANCE_POSITION_KI_PER_S2;
volatile float ball_balance_velocity_kp =
    BALL_BALANCE_VELOCITY_KP_US_PER_MM_S;
volatile float ball_balance_velocity_kd =
    BALL_BALANCE_VELOCITY_KD_US_PER_MM_S2;
volatile float ball_balance_velocity_limit_mm_s =
    BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S;
volatile float ball_balance_target_velocity_mm_s;
volatile float ball_balance_estimated_position_mm;
volatile float ball_balance_measured_velocity_mm_s;
volatile float ball_balance_estimated_acceleration_mm_s2;
volatile float ball_balance_vehicle_acceleration_mps2;
volatile float ball_balance_acceleration_feedforward_us;
volatile float ball_balance_position_feedforward_us =
    SERVO_NEUTRAL_PULSE_US;
volatile float ball_balance_proportional_us;
volatile float ball_balance_derivative_us;
volatile float ball_balance_stopping_distance_mm;
volatile float ball_balance_safe_velocity_mm_s;
volatile float ball_balance_unsaturated_pulse_us =
    SERVO_NEUTRAL_PULSE_US;
volatile uint32_t ball_balance_update_count;
volatile uint16_t ball_balance_servo_pulse_us =
    SERVO_NEUTRAL_PULSE_US;
volatile uint8_t ball_balance_braking_active;
volatile uint8_t ball_balance_output_saturated;
volatile ball_balance_status_t ball_balance_status =
    BALL_BALANCE_DISABLED;

static uint8_t balance_enabled;
static float position_integral_mm_s;
static volatile uint8_t pid_reset_requested;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint16_t quantize_control_pulse(float requested_pulse_us)
{
    int32_t pulse_us;
    int32_t pulse_delta_us;
    int32_t step_us = (int32_t)SERVO_EFFECTIVE_STEP_US;

    pulse_us = (int32_t)(requested_pulse_us + 0.5f);
    if (pulse_us < (int32_t)SERVO_CONTROL_MIN_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us >
             (int32_t)SERVO_CONTROL_MAX_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MAX_PULSE_US;
    }

    pulse_delta_us =
        pulse_us - (int32_t)SERVO_NEUTRAL_PULSE_US;
    if (pulse_delta_us >= 0)
    {
        pulse_delta_us =
            ((pulse_delta_us + step_us / 2) / step_us) *
            step_us;
    }
    else
    {
        pulse_delta_us =
            -(((-pulse_delta_us + step_us / 2) / step_us) *
              step_us);
    }

    pulse_us =
        (int32_t)SERVO_NEUTRAL_PULSE_US + pulse_delta_us;
    if (pulse_us < (int32_t)SERVO_CONTROL_MIN_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us >
             (int32_t)SERVO_CONTROL_MAX_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MAX_PULSE_US;
    }
    return (uint16_t)pulse_us;
}

static void reset_controller_state(void)
{
    position_integral_mm_s = 0.0f;
    ball_balance_target_velocity_mm_s = 0.0f;
    ball_balance_estimated_position_mm = 0.0f;
    ball_balance_measured_velocity_mm_s = 0.0f;
    ball_balance_estimated_acceleration_mm_s2 = 0.0f;
    ball_balance_acceleration_feedforward_us = 0.0f;
    ball_balance_proportional_us = 0.0f;
    ball_balance_derivative_us = 0.0f;
    ball_balance_stopping_distance_mm = 0.0f;
    ball_balance_safe_velocity_mm_s = 0.0f;
    ball_balance_braking_active = 0U;
}

static void set_control_pulse(
    float base_pulse_us,
    float correction_us)
{
    float requested_pulse_us;
    uint16_t pulse_us;

    requested_pulse_us =
        base_pulse_us +
        BALL_BALANCE_SERVO_DIRECTION * correction_us;
    ball_balance_unsaturated_pulse_us = requested_pulse_us;
    ball_balance_output_saturated =
        (requested_pulse_us <
             (float)SERVO_CONTROL_MIN_PULSE_US ||
         requested_pulse_us >
             (float)SERVO_CONTROL_MAX_PULSE_US)
            ? 1U
            : 0U;
    pulse_us = quantize_control_pulse(requested_pulse_us);

    ball_balance_servo_pulse_us = pulse_us;
    servo_set_pulse_us(pulse_us);
}

void ball_balance_init(void)
{
    balance_enabled = 0U;
    ball_balance_target_mm = BALL_BALANCE_TARGET_MM;
    ball_balance_reference_velocity_mm_s = 0.0f;
    ball_balance_position_kp =
        BALL_BALANCE_POSITION_KP_PER_S;
    ball_balance_position_ki =
        BALL_BALANCE_POSITION_KI_PER_S2;
    ball_balance_velocity_kp =
        BALL_BALANCE_VELOCITY_KP_US_PER_MM_S;
    ball_balance_velocity_kd =
        BALL_BALANCE_VELOCITY_KD_US_PER_MM_S2;
    ball_balance_velocity_limit_mm_s =
        BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S;
    ball_balance_vehicle_acceleration_mps2 = 0.0f;
    ball_balance_update_count = 0U;
    pid_reset_requested = 0U;
    ball_balance_status = BALL_BALANCE_DISABLED;
    reset_controller_state();
    ball_balance_position_feedforward_us =
        SERVO_NEUTRAL_PULSE_US;
    set_control_pulse(SERVO_NEUTRAL_PULSE_US, 0.0f);
}

void ball_balance_set_enabled(uint8_t enabled)
{
    enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == balance_enabled)
    {
        return;
    }

    balance_enabled = enabled;
    if (enabled == 0U)
    {
        ball_balance_target_mm = BALL_BALANCE_TARGET_MM;
        ball_balance_reference_velocity_mm_s = 0.0f;
        ball_balance_vehicle_acceleration_mps2 = 0.0f;
    }
    reset_controller_state();
    ball_balance_position_feedforward_us =
        SERVO_NEUTRAL_PULSE_US;
    set_control_pulse(SERVO_NEUTRAL_PULSE_US, 0.0f);
    ball_balance_status =
        (enabled != 0U) ? BALL_BALANCE_WAITING : BALL_BALANCE_DISABLED;
}

void ball_balance_set_vehicle_acceleration(float acceleration_mps2)
{
    ball_balance_vehicle_acceleration_mps2 = clamp_float(
        acceleration_mps2,
        -BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2,
        BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2);
}

void ball_balance_set_reference(
    float position_mm,
    float velocity_mm_s)
{
    ball_balance_target_mm = clamp_float(
        position_mm,
        -BALL_BALANCE_REFERENCE_POSITION_LIMIT_MM,
        BALL_BALANCE_REFERENCE_POSITION_LIMIT_MM);
    ball_balance_reference_velocity_mm_s = clamp_float(
        velocity_mm_s,
        -BALL_BALANCE_REFERENCE_VELOCITY_LIMIT_MM_S,
        BALL_BALANCE_REFERENCE_VELOCITY_LIMIT_MM_S);
}

uint8_t ball_balance_set_cascade_gains(
    float position_kp,
    float position_ki,
    float velocity_kp,
    float velocity_kd,
    float velocity_limit_mm_s)
{
    if (position_kp < BALL_BALANCE_POSITION_KP_MIN ||
        position_kp > BALL_BALANCE_POSITION_KP_MAX ||
        position_ki < BALL_BALANCE_POSITION_KI_MIN ||
        position_ki > BALL_BALANCE_POSITION_KI_MAX ||
        velocity_kp < BALL_BALANCE_VELOCITY_KP_MIN ||
        velocity_kp > BALL_BALANCE_VELOCITY_KP_MAX ||
        velocity_kd < BALL_BALANCE_VELOCITY_KD_MIN ||
        velocity_kd > BALL_BALANCE_VELOCITY_KD_MAX ||
        velocity_limit_mm_s <
            BALL_BALANCE_VELOCITY_LIMIT_MIN_MM_S ||
        velocity_limit_mm_s >
            BALL_BALANCE_VELOCITY_LIMIT_MAX_MM_S)
    {
        return 0U;
    }

    ball_balance_position_kp = position_kp;
    ball_balance_position_ki = position_ki;
    ball_balance_velocity_kp = velocity_kp;
    ball_balance_velocity_kd = velocity_kd;
    ball_balance_velocity_limit_mm_s = velocity_limit_mm_s;
    pid_reset_requested = 1U;
    return 1U;
}

void ball_balance_reset_pid_gains(void)
{
    (void)ball_balance_set_cascade_gains(
        BALL_BALANCE_POSITION_KP_PER_S,
        BALL_BALANCE_POSITION_KI_PER_S2,
        BALL_BALANCE_VELOCITY_KP_US_PER_MM_S,
        BALL_BALANCE_VELOCITY_KD_US_PER_MM_S2,
        BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S);
}

void ball_balance_update(void)
{
    float position_mm;
    float velocity_mm_s;
    float acceleration_mm_s2;
    float target_position_mm;
    float reference_velocity_mm_s;
    float raw_position_error_mm;
    float position_error_mm;
    float target_velocity_mm_s;
    float target_velocity_unsaturated_mm_s;
    float velocity_error_mm_s;
    float correction_us;
    float correction_unsaturated_us;
    float acceleration_feedforward_us;
    float position_integral_candidate_mm_s;
    float speed_excess_mm_s;
    float braking_target_magnitude_mm_s;
    uint8_t integral_candidate_allowed;
    ball_braking_result_t braking_result;

    ball_balance_update_count++;

    if (pid_reset_requested != 0U)
    {
        pid_reset_requested = 0U;
        reset_controller_state();
    }

    if (balance_enabled == 0U)
    {
        return;
    }

    if (ball_state_observer.valid == 0U)
    {
        ball_balance_status =
            (ball_state_observer.last_frame_count == 0U) ? BALL_BALANCE_WAITING : BALL_BALANCE_STALE;
        reset_controller_state();
        ball_balance_position_feedforward_us =
            SERVO_NEUTRAL_PULSE_US;
        set_control_pulse(SERVO_NEUTRAL_PULSE_US, 0.0f);
        return;
    }

    position_mm = ball_state_observer.position_mm;
    velocity_mm_s = ball_state_observer.velocity_mm_s;
    acceleration_mm_s2 =
        ball_state_observer.acceleration_mm_s2;
    target_position_mm = ball_balance_target_mm;
    reference_velocity_mm_s =
        ball_balance_reference_velocity_mm_s;
    ball_balance_position_feedforward_us =
        ball_position_feedforward_us(position_mm);

    raw_position_error_mm = target_position_mm - position_mm;
    position_error_mm = raw_position_error_mm;
    if (position_error_mm > -BALL_BALANCE_POSITION_DEADBAND_MM &&
        position_error_mm < BALL_BALANCE_POSITION_DEADBAND_MM)
    {
        position_error_mm = 0.0f;
    }

    position_integral_candidate_mm_s = clamp_float(
        position_integral_mm_s +
            position_error_mm * BALL_OBSERVER_DT_S,
        -BALL_BALANCE_POSITION_INTEGRAL_LIMIT_MM_S,
        BALL_BALANCE_POSITION_INTEGRAL_LIMIT_MM_S);

    target_velocity_unsaturated_mm_s =
        reference_velocity_mm_s +
        ball_balance_position_kp * position_error_mm +
        ball_balance_position_ki *
            position_integral_candidate_mm_s;
    target_velocity_mm_s = clamp_float(
        target_velocity_unsaturated_mm_s,
        -ball_balance_velocity_limit_mm_s,
        ball_balance_velocity_limit_mm_s);
    integral_candidate_allowed =
        (target_velocity_mm_s ==
            target_velocity_unsaturated_mm_s ||
         (target_velocity_unsaturated_mm_s >
              target_velocity_mm_s &&
          position_error_mm < 0.0f) ||
         (target_velocity_unsaturated_mm_s <
              target_velocity_mm_s &&
          position_error_mm > 0.0f))
            ? 1U
            : 0U;

    ball_balance_stopping_distance_mm = 0.0f;
    ball_balance_safe_velocity_mm_s = 0.0f;
    ball_balance_braking_active = 0U;
    if (reference_velocity_mm_s >
            -BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S &&
        reference_velocity_mm_s <
            BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S &&
        ball_braking_calculate(
            raw_position_error_mm,
            velocity_mm_s,
            BALL_BALANCE_STOPPING_ACCEL_MM_S2,
            BALL_BALANCE_EFFECTIVE_DELAY_S,
            BALL_BALANCE_BRAKING_MARGIN_MM,
            BALL_BALANCE_BRAKE_RELEASE_MM_S,
            &braking_result) != 0U)
    {
        ball_balance_stopping_distance_mm =
            braking_result.stopping_distance_mm;
        ball_balance_safe_velocity_mm_s =
            braking_result.safe_velocity_mm_s;
        if (braking_result.braking_required != 0U)
        {
            speed_excess_mm_s =
                ((velocity_mm_s < 0.0f) ?
                     -velocity_mm_s : velocity_mm_s) -
                braking_result.safe_velocity_mm_s;
            braking_target_magnitude_mm_s =
                ((target_velocity_mm_s < 0.0f) ?
                     -target_velocity_mm_s :
                     target_velocity_mm_s) -
                BALL_BALANCE_BRAKING_EXCESS_GAIN *
                    speed_excess_mm_s;
            if (braking_target_magnitude_mm_s < 0.0f)
            {
                braking_target_magnitude_mm_s = 0.0f;
            }
            target_velocity_mm_s =
                (raw_position_error_mm < 0.0f) ?
                    -braking_target_magnitude_mm_s :
                    braking_target_magnitude_mm_s;
            ball_balance_braking_active = 1U;
        }
    }

    velocity_error_mm_s =
        target_velocity_mm_s - velocity_mm_s;
    if (velocity_error_mm_s >
            -BALL_BALANCE_VELOCITY_DEADBAND_MM_S &&
        velocity_error_mm_s <
            BALL_BALANCE_VELOCITY_DEADBAND_MM_S)
    {
        velocity_error_mm_s = 0.0f;
    }

    acceleration_feedforward_us = clamp_float(
        BALL_BALANCE_ACCELERATION_FF_DIRECTION *
            BALL_BALANCE_ACCELERATION_FF_US_PER_MPS2 *
            ball_balance_vehicle_acceleration_mps2,
        -BALL_BALANCE_ACCELERATION_FF_LIMIT_US,
        BALL_BALANCE_ACCELERATION_FF_LIMIT_US);
    ball_balance_proportional_us =
        ball_balance_velocity_kp * velocity_error_mm_s;
    ball_balance_derivative_us =
        -ball_balance_velocity_kd * acceleration_mm_s2;
    correction_unsaturated_us =
        ball_balance_proportional_us +
        ball_balance_derivative_us +
        acceleration_feedforward_us;
    correction_us = correction_unsaturated_us;

    ball_balance_target_velocity_mm_s = target_velocity_mm_s;
    ball_balance_estimated_position_mm = position_mm;
    ball_balance_measured_velocity_mm_s = velocity_mm_s;
    ball_balance_estimated_acceleration_mm_s2 =
        acceleration_mm_s2;
    ball_balance_acceleration_feedforward_us =
        acceleration_feedforward_us;
    ball_balance_status = BALL_BALANCE_ACTIVE;
    set_control_pulse(
        ball_balance_position_feedforward_us,
        correction_us);
    if (integral_candidate_allowed != 0U &&
        (ball_balance_output_saturated == 0U ||
         (ball_balance_unsaturated_pulse_us >
              (float)SERVO_CONTROL_MAX_PULSE_US &&
          BALL_BALANCE_SERVO_DIRECTION *
                  position_error_mm <
              0.0f) ||
         (ball_balance_unsaturated_pulse_us <
              (float)SERVO_CONTROL_MIN_PULSE_US &&
          BALL_BALANCE_SERVO_DIRECTION *
                  position_error_mm >
              0.0f)))
    {
        position_integral_mm_s =
            position_integral_candidate_mm_s;
    }
}

#include "ball_balance.h"

#include "servo.h"
#include "uart_callback.h"

#include <math.h>

#define BALL_BALANCE_MIN_DT_MS             (5UL)
#define BALL_BALANCE_MAX_DT_MS             (200UL)
#define BALL_BALANCE_POSITION_DEADBAND_MM   (1.0f)
#define BALL_BALANCE_VELOCITY_DEADBAND_MM_S (2.0f)
#define BALL_BALANCE_POSITION_INTEGRAL_LIMIT_MM_S (300.0f)
#define BALL_BALANCE_VELOCITY_FILTER_ALPHA  (0.35f)
#define BALL_BALANCE_BRAKING_ACCEL_MM_S2    (180.0f)
#define BALL_BALANCE_CONTROL_MIN_PULSE_US   (800U)
#define BALL_BALANCE_CONTROL_MAX_PULSE_US   (1800U)
#define BALL_BALANCE_MIN_CORRECTION_US      \
    ((float)BALL_BALANCE_CONTROL_MIN_PULSE_US - \
     (float)SERVO_NEUTRAL_PULSE_US)
#define BALL_BALANCE_MAX_CORRECTION_US      \
    ((float)BALL_BALANCE_CONTROL_MAX_PULSE_US - \
     (float)SERVO_NEUTRAL_PULSE_US)
#define BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S (1.0f)

extern volatile unsigned long tick_ms;

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
volatile float ball_balance_measured_velocity_mm_s;
volatile float ball_balance_vehicle_acceleration_mps2;
volatile float ball_balance_acceleration_feedforward_us;
volatile uint16_t ball_balance_servo_pulse_us =
    SERVO_NEUTRAL_PULSE_US;
volatile ball_balance_status_t ball_balance_status =
    BALL_BALANCE_DISABLED;

static uint8_t balance_enabled;
static uint8_t have_previous_sample;
static uint8_t have_previous_velocity_error;
static uint32_t previous_frame_count;
static uint32_t previous_sample_ms;
static float filtered_velocity_mm_s;
static float position_integral_mm_s;
static float previous_velocity_error_mm_s;
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
    if (pulse_us < (int32_t)BALL_BALANCE_CONTROL_MIN_PULSE_US)
    {
        pulse_us = BALL_BALANCE_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us >
             (int32_t)BALL_BALANCE_CONTROL_MAX_PULSE_US)
    {
        pulse_us = BALL_BALANCE_CONTROL_MAX_PULSE_US;
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
    if (pulse_us < (int32_t)BALL_BALANCE_CONTROL_MIN_PULSE_US)
    {
        pulse_us = BALL_BALANCE_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us >
             (int32_t)BALL_BALANCE_CONTROL_MAX_PULSE_US)
    {
        pulse_us = BALL_BALANCE_CONTROL_MAX_PULSE_US;
    }
    return (uint16_t)pulse_us;
}

static void reset_controller_state(void)
{
    have_previous_sample = 0U;
    have_previous_velocity_error = 0U;
    previous_frame_count = vision_ball_frame_count;
    previous_sample_ms = (uint32_t)tick_ms;
    filtered_velocity_mm_s = 0.0f;
    position_integral_mm_s = 0.0f;
    previous_velocity_error_mm_s = 0.0f;
    ball_balance_target_velocity_mm_s = 0.0f;
    ball_balance_measured_velocity_mm_s = 0.0f;
    ball_balance_acceleration_feedforward_us = 0.0f;
}

static void set_control_pulse(float correction_us)
{
    uint16_t pulse_us;

    correction_us = clamp_float(
        correction_us,
        BALL_BALANCE_MIN_CORRECTION_US,
        BALL_BALANCE_MAX_CORRECTION_US);
    pulse_us = quantize_control_pulse(
        (float)SERVO_NEUTRAL_PULSE_US +
        BALL_BALANCE_SERVO_DIRECTION * correction_us);

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
    pid_reset_requested = 0U;
    ball_balance_status = BALL_BALANCE_DISABLED;
    reset_controller_state();
    set_control_pulse(0.0f);
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
    set_control_pulse(0.0f);
    ball_balance_status =
        (enabled != 0U) ? BALL_BALANCE_WAITING :
                          BALL_BALANCE_DISABLED;
}

void ball_balance_set_vehicle_acceleration(float acceleration_mps2)
{
    ball_balance_vehicle_acceleration_mps2 = clamp_float(
        acceleration_mps2,
        -BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2,
        BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2);
}

void ball_balance_set_vehicle_acceleration_from_raw_ay(
    float raw_ay_g)
{
    ball_balance_set_vehicle_acceleration(
        raw_ay_g * 9.80665f);
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
    uint32_t now_ms;
    uint32_t age_ms;
    uint32_t frame_count_before;
    uint32_t frame_count;
    uint32_t sample_ms;
    uint32_t dt_ms;
    float position_mm;
    float velocity_mm_s;
    float target_position_mm;
    float reference_velocity_mm_s;
    float position_error_mm;
    float target_velocity_mm_s;
    float target_velocity_unsaturated_mm_s;
    float braking_velocity_limit_mm_s;
    float velocity_error_mm_s;
    float velocity_error_derivative_mm_s2;
    float correction_us;
    float correction_unsaturated_us;
    float acceleration_feedforward_us;
    float position_integral_candidate_mm_s;
    float dt_s;

    if (pid_reset_requested != 0U)
    {
        pid_reset_requested = 0U;
        reset_controller_state();
    }

    if (balance_enabled == 0U)
    {
        return;
    }

    now_ms = (uint32_t)tick_ms;
    if (vision_ball_position_valid == 0U)
    {
        ball_balance_status = BALL_BALANCE_WAITING;
        reset_controller_state();
        set_control_pulse(0.0f);
        return;
    }

    age_ms = now_ms - vision_ball_last_update_ms;
    if (age_ms > BALL_BALANCE_VISION_TIMEOUT_MS)
    {
        ball_balance_status = BALL_BALANCE_STALE;
        reset_controller_state();
        set_control_pulse(0.0f);
        return;
    }

    do
    {
        frame_count_before = vision_ball_frame_count;
        position_mm = vision_ball_position_mm;
        velocity_mm_s = vision_ball_velocity_mm_s;
        target_position_mm = ball_balance_target_mm;
        reference_velocity_mm_s =
            ball_balance_reference_velocity_mm_s;
        sample_ms = vision_ball_last_update_ms;
        frame_count = vision_ball_frame_count;
    } while (frame_count_before != frame_count);

    if (frame_count == previous_frame_count)
    {
        return;
    }
    previous_frame_count = frame_count;

    if (have_previous_sample == 0U)
    {
        have_previous_sample = 1U;
        filtered_velocity_mm_s = velocity_mm_s;
        previous_sample_ms = sample_ms;
        dt_ms = BALL_BALANCE_MIN_DT_MS;
    }
    else
    {
        dt_ms = sample_ms - previous_sample_ms;
        if (dt_ms < BALL_BALANCE_MIN_DT_MS)
        {
            dt_ms = BALL_BALANCE_MIN_DT_MS;
        }
        else if (dt_ms > BALL_BALANCE_MAX_DT_MS)
        {
            dt_ms = BALL_BALANCE_MAX_DT_MS;
        }
        filtered_velocity_mm_s +=
            BALL_BALANCE_VELOCITY_FILTER_ALPHA *
            (velocity_mm_s - filtered_velocity_mm_s);
    }
    dt_s = (float)dt_ms * 0.001f;

    position_error_mm = target_position_mm - position_mm;
    if (position_error_mm > -BALL_BALANCE_POSITION_DEADBAND_MM &&
        position_error_mm < BALL_BALANCE_POSITION_DEADBAND_MM)
    {
        position_error_mm = 0.0f;
    }

    position_integral_candidate_mm_s = clamp_float(
        position_integral_mm_s + position_error_mm * dt_s,
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
    if (target_velocity_mm_s ==
            target_velocity_unsaturated_mm_s ||
        (target_velocity_unsaturated_mm_s >
             target_velocity_mm_s &&
         position_error_mm < 0.0f) ||
        (target_velocity_unsaturated_mm_s <
             target_velocity_mm_s &&
         position_error_mm > 0.0f))
    {
        position_integral_mm_s =
            position_integral_candidate_mm_s;
    }

    if (reference_velocity_mm_s >
            -BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S &&
        reference_velocity_mm_s <
            BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S)
    {
        braking_velocity_limit_mm_s = sqrtf(
            2.0f * BALL_BALANCE_BRAKING_ACCEL_MM_S2 *
            ((position_error_mm < 0.0f) ?
                 -position_error_mm : position_error_mm));
        if (braking_velocity_limit_mm_s <
            ball_balance_velocity_limit_mm_s)
        {
            target_velocity_mm_s = clamp_float(
                target_velocity_mm_s,
                -braking_velocity_limit_mm_s,
                braking_velocity_limit_mm_s);
        }
    }

    velocity_error_mm_s =
        target_velocity_mm_s - filtered_velocity_mm_s;
    if (velocity_error_mm_s >
            -BALL_BALANCE_VELOCITY_DEADBAND_MM_S &&
        velocity_error_mm_s <
            BALL_BALANCE_VELOCITY_DEADBAND_MM_S)
    {
        velocity_error_mm_s = 0.0f;
    }

    if (have_previous_velocity_error == 0U)
    {
        have_previous_velocity_error = 1U;
        velocity_error_derivative_mm_s2 = 0.0f;
    }
    else
    {
        velocity_error_derivative_mm_s2 =
            (velocity_error_mm_s -
             previous_velocity_error_mm_s) / dt_s;
    }
    previous_velocity_error_mm_s = velocity_error_mm_s;

    acceleration_feedforward_us = clamp_float(
        BALL_BALANCE_ACCELERATION_FF_DIRECTION *
            BALL_BALANCE_ACCELERATION_FF_US_PER_MPS2 *
            ball_balance_vehicle_acceleration_mps2,
        -BALL_BALANCE_ACCELERATION_FF_LIMIT_US,
        BALL_BALANCE_ACCELERATION_FF_LIMIT_US);
    correction_unsaturated_us =
        ball_balance_velocity_kp * velocity_error_mm_s +
        ball_balance_velocity_kd *
            velocity_error_derivative_mm_s2 +
        acceleration_feedforward_us;
    correction_us = clamp_float(
        correction_unsaturated_us,
        BALL_BALANCE_MIN_CORRECTION_US,
        BALL_BALANCE_MAX_CORRECTION_US);

    previous_sample_ms = sample_ms;
    ball_balance_target_velocity_mm_s = target_velocity_mm_s;
    ball_balance_measured_velocity_mm_s =
        filtered_velocity_mm_s;
    ball_balance_acceleration_feedforward_us =
        acceleration_feedforward_us;
    ball_balance_status = BALL_BALANCE_ACTIVE;
    set_control_pulse(correction_us);
}

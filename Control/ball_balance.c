#include "ball_balance.h"

#include "servo.h"
#include "uart_callback.h"

#define BALL_BALANCE_MIN_DT_MS             (5UL)
#define BALL_BALANCE_MAX_DT_MS             (200UL)
#define BALL_BALANCE_DEADBAND_MM            (1.0f)
#define BALL_BALANCE_INTEGRAL_LIMIT_MM_S    (1000.0f)
#define BALL_BALANCE_VELOCITY_FILTER_ALPHA  (0.25f)

extern volatile unsigned long tick_ms;

volatile float ball_balance_target_mm =
    BALL_BALANCE_DEFAULT_TARGET_MM;
volatile uint16_t ball_balance_servo_pulse_us =
    SERVO_NEUTRAL_PULSE_US;
volatile ball_balance_status_t ball_balance_status =
    BALL_BALANCE_DISABLED;

static uint8_t balance_enabled;
static uint8_t have_previous_sample;
static uint32_t previous_frame_count;
static uint32_t previous_sample_ms;
static float previous_position_mm;
static float filtered_velocity_mm_s;
static float integral_mm_s;

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

static void reset_controller_state(void)
{
    have_previous_sample = 0U;
    previous_frame_count = vision_ball_frame_count;
    previous_sample_ms = (uint32_t)tick_ms;
    previous_position_mm = vision_ball_position_mm;
    filtered_velocity_mm_s = 0.0f;
    integral_mm_s = 0.0f;
}

static void set_balance_pulse(float correction_us)
{
    float requested_pulse =
        (float)SERVO_NEUTRAL_PULSE_US +
        BALL_BALANCE_SERVO_DIRECTION * correction_us;
    uint16_t pulse_us;

    requested_pulse = clamp_float(
        requested_pulse,
        (float)BALL_BALANCE_SERVO_MIN_US,
        (float)BALL_BALANCE_SERVO_MAX_US);
    pulse_us = (uint16_t)(requested_pulse + 0.5f);

    ball_balance_servo_pulse_us = pulse_us;
    servo_set_pulse_us(pulse_us);
}

void ball_balance_init(void)
{
    balance_enabled = 0U;
    reset_controller_state();
    ball_balance_servo_pulse_us = SERVO_NEUTRAL_PULSE_US;
    ball_balance_status = BALL_BALANCE_DISABLED;
    servo_set_pulse_us(SERVO_NEUTRAL_PULSE_US);
}

void ball_balance_set_enabled(uint8_t enabled)
{
    enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == balance_enabled)
    {
        return;
    }

    balance_enabled = enabled;
    reset_controller_state();
    set_balance_pulse(0.0f);
    ball_balance_status =
        (enabled != 0U) ? BALL_BALANCE_WAITING :
                          BALL_BALANCE_DISABLED;
}

void ball_balance_set_target_mm(float target_mm)
{
    ball_balance_target_mm = target_mm;
    integral_mm_s = 0.0f;
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
    float error_mm;
    float velocity_mm_s;
    float correction_us;
    float dt_s;

    if (balance_enabled == 0U)
    {
        return;
    }

    now_ms = (uint32_t)tick_ms;
    if (vision_ball_position_valid == 0U)
    {
        ball_balance_status = BALL_BALANCE_WAITING;
        reset_controller_state();
        set_balance_pulse(0.0f);
        return;
    }

    age_ms = now_ms - vision_ball_last_update_ms;
    if (age_ms > BALL_BALANCE_VISION_TIMEOUT_MS)
    {
        ball_balance_status = BALL_BALANCE_STALE;
        reset_controller_state();
        set_balance_pulse(0.0f);
        return;
    }

    do
    {
        frame_count_before = vision_ball_frame_count;
        position_mm = vision_ball_position_mm;
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
        previous_position_mm = position_mm;
        previous_sample_ms = sample_ms;
        ball_balance_status = BALL_BALANCE_ACTIVE;
        return;
    }

    dt_ms = sample_ms - previous_sample_ms;
    if (dt_ms < BALL_BALANCE_MIN_DT_MS)
    {
        dt_ms = BALL_BALANCE_MIN_DT_MS;
    }
    else if (dt_ms > BALL_BALANCE_MAX_DT_MS)
    {
        dt_ms = BALL_BALANCE_MAX_DT_MS;
    }
    dt_s = (float)dt_ms * 0.001f;

    velocity_mm_s =
        (position_mm - previous_position_mm) / dt_s;
    filtered_velocity_mm_s +=
        BALL_BALANCE_VELOCITY_FILTER_ALPHA *
        (velocity_mm_s - filtered_velocity_mm_s);

    error_mm = ball_balance_target_mm - position_mm;
    if (error_mm > -BALL_BALANCE_DEADBAND_MM &&
        error_mm < BALL_BALANCE_DEADBAND_MM)
    {
        error_mm = 0.0f;
    }

    integral_mm_s += error_mm * dt_s;
    integral_mm_s = clamp_float(
        integral_mm_s,
        -BALL_BALANCE_INTEGRAL_LIMIT_MM_S,
        BALL_BALANCE_INTEGRAL_LIMIT_MM_S);

    correction_us =
        BALL_BALANCE_KP_US_PER_MM * error_mm +
        BALL_BALANCE_KI_US_PER_MM_S * integral_mm_s -
        BALL_BALANCE_KD_US_PER_MM_PER_S *
            filtered_velocity_mm_s;

    previous_position_mm = position_mm;
    previous_sample_ms = sample_ms;
    ball_balance_status = BALL_BALANCE_ACTIVE;
    set_balance_pulse(correction_us);
}

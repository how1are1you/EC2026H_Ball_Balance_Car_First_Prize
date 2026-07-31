#include "ball_static_task.h"

#include "ball_balance.h"
#include "servo.h"
#include "uart_callback.h"

#include <stdio.h>

#define BALL_STATIC_POS_TARGET_MM (50.0f)
#define BALL_STATIC_NEG_TARGET_MM (-50.0f)
#define BALL_STATIC_START_TOLERANCE_MM (5.0f)
#define BALL_STATIC_READY_TIME_MS (200UL)
#define BALL_STATIC_VISION_TIMEOUT_MS (200UL)
#define BALL_STATIC_SETTLE_TOLERANCE_MM (5.0f)
#define BALL_STATIC_SETTLE_VELOCITY_MM_S (10.0f)
#define BALL_STATIC_POS_SETTLE_TIME_MS (100UL)
#define BALL_STATIC_NEG_SETTLE_TIME_MS (300UL)
#define BALL_STATIC_DEFAULT_UP_PULSE_US (1550U)
#define BALL_STATIC_DEFAULT_DOWN_PULSE_US (1050U)
#define BALL_STATIC_DEFAULT_HOLD_DELTA_US (30U)
#define BALL_STATIC_DEFAULT_POS_SWITCH_MM (9.2f)
#define BALL_STATIC_DEFAULT_NEG_SWITCH_MM (-4.0f)
#define BALL_STATIC_DEFAULT_DEADBAND_MM (2.5f)
#define BALL_STATIC_DEFAULT_VELOCITY_MM_S (10.0f)

extern volatile unsigned long tick_ms;

volatile ball_static_state_t ball_static_state =
    BALL_STATIC_READY;
volatile ball_static_fault_t ball_static_fault =
    BALL_STATIC_FAULT_NONE;
volatile uint32_t ball_static_elapsed_ms;
volatile float ball_static_positive_max_error_mm;
volatile float ball_static_negative_max_error_mm;
volatile uint8_t ball_static_ready;
volatile uint16_t ball_static_up_pulse_us =
    BALL_STATIC_DEFAULT_UP_PULSE_US;
volatile uint16_t ball_static_down_pulse_us =
    BALL_STATIC_DEFAULT_DOWN_PULSE_US;
volatile uint16_t ball_static_hold_delta_us =
    BALL_STATIC_DEFAULT_HOLD_DELTA_US;
volatile uint16_t ball_static_command_pulse_us =
    SERVO_NEUTRAL_PULSE_US;
volatile float ball_static_target_mm;
volatile float ball_static_positive_switch_mm =
    BALL_STATIC_DEFAULT_POS_SWITCH_MM;
volatile float ball_static_negative_switch_mm =
    BALL_STATIC_DEFAULT_NEG_SWITCH_MM;
volatile float ball_static_hold_deadband_mm =
    BALL_STATIC_DEFAULT_DEADBAND_MM;
volatile float ball_static_velocity_threshold_mm_s =
    BALL_STATIC_DEFAULT_VELOCITY_MM_S;

static uint32_t task_start_ms;
static uint32_t ready_start_ms;
static uint32_t settle_start_ms;
static float positive_peak_mm;

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t vision_is_fresh(uint32_t now_ms)
{
    return (vision_ball_position_valid != 0U) &&
           ((uint32_t)(now_ms - vision_ball_last_update_ms) <=
            BALL_STATIC_VISION_TIMEOUT_MS);
}

static uint8_t target_is_stable(
    float position_mm,
    float velocity_mm_s,
    float target_mm)
{
    return (absolute_float(position_mm - target_mm) <=
                BALL_STATIC_SETTLE_TOLERANCE_MM &&
            absolute_float(velocity_mm_s) <=
                BALL_STATIC_SETTLE_VELOCITY_MM_S)
               ? 1U
               : 0U;
}

static void apply_pulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_CONTROL_MIN_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_CONTROL_MAX_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MAX_PULSE_US;
    }
    ball_static_command_pulse_us = pulse_us;
    servo_set_pulse_us(pulse_us);
}

static void set_fault(
    ball_static_fault_t fault,
    uint32_t now_ms)
{
    ball_static_fault = fault;
    ball_static_state = BALL_STATIC_FAULT;
    ball_static_elapsed_ms = now_ms - task_start_ms;
    ball_static_ready = 0U;
    apply_pulse(SERVO_NEUTRAL_PULSE_US);
}

void ball_static_reset_open_loop_config(void)
{
    ball_static_up_pulse_us =
        BALL_STATIC_DEFAULT_UP_PULSE_US;
    ball_static_down_pulse_us =
        BALL_STATIC_DEFAULT_DOWN_PULSE_US;
    ball_static_positive_switch_mm =
        BALL_STATIC_DEFAULT_POS_SWITCH_MM;
    ball_static_negative_switch_mm =
        BALL_STATIC_DEFAULT_NEG_SWITCH_MM;
    ball_static_hold_delta_us =
        BALL_STATIC_DEFAULT_HOLD_DELTA_US;
    ball_static_hold_deadband_mm =
        BALL_STATIC_DEFAULT_DEADBAND_MM;
    ball_static_velocity_threshold_mm_s =
        BALL_STATIC_DEFAULT_VELOCITY_MM_S;
}

uint8_t ball_static_set_open_loop_config(
    uint16_t up_pulse_us,
    uint16_t down_pulse_us,
    float positive_switch_mm,
    float negative_switch_mm,
    uint16_t hold_delta_us,
    float hold_deadband_mm,
    float velocity_threshold_mm_s)
{
    if (up_pulse_us < SERVO_CONTROL_MIN_PULSE_US ||
        up_pulse_us > SERVO_CONTROL_MAX_PULSE_US ||
        down_pulse_us < SERVO_CONTROL_MIN_PULSE_US ||
        down_pulse_us > SERVO_CONTROL_MAX_PULSE_US ||
        up_pulse_us == SERVO_NEUTRAL_PULSE_US ||
        down_pulse_us == SERVO_NEUTRAL_PULSE_US ||
        ((up_pulse_us > SERVO_NEUTRAL_PULSE_US) ==
         (down_pulse_us > SERVO_NEUTRAL_PULSE_US)) ||
        positive_switch_mm < 0.0f ||
        positive_switch_mm > 48.0f ||
        negative_switch_mm > 48.0f ||
        negative_switch_mm < -48.0f ||
        hold_delta_us < SERVO_EFFECTIVE_STEP_US ||
        hold_delta_us > 120U ||
        hold_deadband_mm < 1.0f ||
        hold_deadband_mm > 8.0f ||
        velocity_threshold_mm_s < 5.0f ||
        velocity_threshold_mm_s > 30.0f)
    {
        return 0U;
    }

    ball_static_up_pulse_us = up_pulse_us;
    ball_static_down_pulse_us = down_pulse_us;
    ball_static_positive_switch_mm = positive_switch_mm;
    ball_static_negative_switch_mm = negative_switch_mm;
    ball_static_hold_delta_us = hold_delta_us;
    ball_static_hold_deadband_mm = hold_deadband_mm;
    ball_static_velocity_threshold_mm_s =
        velocity_threshold_mm_s;
    return 1U;
}

void ball_static_task_init(void)
{
    ball_static_reset_open_loop_config();
    ball_static_task_reset();
}

void ball_static_task_reset(void)
{
    ball_static_state = BALL_STATIC_READY;
    ball_static_fault = BALL_STATIC_FAULT_NONE;
    ball_static_elapsed_ms = 0U;
    ball_static_positive_max_error_mm = 0.0f;
    ball_static_negative_max_error_mm = 0.0f;
    ball_static_ready = 0U;
    ball_static_target_mm = 0.0f;
    task_start_ms = 0U;
    ready_start_ms = 0U;
    settle_start_ms = 0U;
    positive_peak_mm = 0.0f;
    ball_balance_set_reference(0.0f, 0.0f);
    apply_pulse(SERVO_NEUTRAL_PULSE_US);
}

uint8_t ball_static_task_start(void)
{
    uint32_t now_ms = (uint32_t)tick_ms;

    if (ball_static_state != BALL_STATIC_READY)
    {
        ball_static_task_reset();
        return 0U;
    }
    if (ball_static_ready == 0U)
    {
        ball_static_fault =
            vision_is_fresh(now_ms) ? BALL_STATIC_FAULT_START_POSITION : BALL_STATIC_FAULT_VISION;
        return 0U;
    }

    task_start_ms = now_ms;
    positive_peak_mm = vision_ball_position_mm;
    settle_start_ms = 0U;
    ball_static_elapsed_ms = 0U;
    ball_static_positive_max_error_mm = 0.0f;
    ball_static_negative_max_error_mm = 0.0f;
    ball_static_fault = BALL_STATIC_FAULT_NONE;
    ball_static_ready = 0U;
    ball_static_target_mm = BALL_STATIC_POS_TARGET_MM;
    ball_static_state = BALL_STATIC_MOVE_POS;
    ball_balance_set_reference(BALL_STATIC_POS_TARGET_MM, 0.0f);
    return 1U;
}

void ball_static_task_stop(void)
{
    ball_static_task_reset();
}

uint8_t ball_static_task_controller_enabled(void)
{
    return (ball_static_state == BALL_STATIC_MOVE_POS ||
            ball_static_state == BALL_STATIC_HOLD_POS ||
            ball_static_state == BALL_STATIC_MOVE_NEG ||
            ball_static_state == BALL_STATIC_HOLD_NEG ||
            ball_static_state == BALL_STATIC_PID_HOLD ||
            ball_static_state == BALL_STATIC_DONE)
               ? 1U
               : 0U;
}

uint8_t ball_static_task_is_running(void)
{
    return (ball_static_state == BALL_STATIC_MOVE_POS ||
            ball_static_state == BALL_STATIC_HOLD_POS ||
            ball_static_state == BALL_STATIC_MOVE_NEG ||
            ball_static_state == BALL_STATIC_HOLD_NEG ||
            ball_static_state == BALL_STATIC_PID_HOLD ||
            ball_static_state == BALL_STATIC_DONE)
               ? 1U
               : 0U;
}

void ball_static_task_service(void)
{
    static uint32_t last_report_ms;
    uint32_t now_ms = (uint32_t)tick_ms;

    if (ball_static_state == BALL_STATIC_READY ||
        (uint32_t)(now_ms - last_report_ms) < 200UL)
    {
        return;
    }
    last_report_ms = now_ms;
    printf(
        "BOL,%lu,%u,%d,%d,%u,%u,%d,%d\r\n",
        (unsigned long)ball_static_elapsed_ms,
        (unsigned int)ball_static_state,
        (int)(vision_ball_position_mm * 10.0f),
        (int)(vision_ball_velocity_mm_s * 10.0f),
        (unsigned int)servo_get_pulse_us(),
        (unsigned int)ball_static_fault,
        (int)(ball_static_positive_max_error_mm * 10.0f),
        (int)(ball_static_negative_max_error_mm * 10.0f));
}

void ball_static_task_update(void)
{
    uint32_t now_ms = (uint32_t)tick_ms;
    float position_mm = vision_ball_position_mm;
    float velocity_mm_s = vision_ball_velocity_mm_s;
    float endpoint_error_mm;

    if (ball_static_state == BALL_STATIC_READY)
    {
        ball_static_target_mm = 0.0f;
        apply_pulse(SERVO_NEUTRAL_PULSE_US);
        if (vision_is_fresh(now_ms) != 0U &&
            absolute_float(position_mm) <=
                BALL_STATIC_START_TOLERANCE_MM)
        {
            if (ready_start_ms == 0U)
            {
                ready_start_ms = now_ms;
            }
            if ((uint32_t)(now_ms - ready_start_ms) >=
                BALL_STATIC_READY_TIME_MS)
            {
                ball_static_ready = 1U;
                ball_static_fault = BALL_STATIC_FAULT_NONE;
            }
        }
        else
        {
            ready_start_ms = 0U;
            ball_static_ready = 0U;
        }
        return;
    }

    if (ball_static_state == BALL_STATIC_FAULT)
    {
        apply_pulse(SERVO_NEUTRAL_PULSE_US);
        return;
    }

    ball_static_elapsed_ms = now_ms - task_start_ms;
    if (ball_static_state != BALL_STATIC_DONE &&
        vision_is_fresh(now_ms) == 0U)
    {
        set_fault(BALL_STATIC_FAULT_VISION, now_ms);
        return;
    }

    switch (ball_static_state)
    {
    case BALL_STATIC_MOVE_POS:
        ball_static_target_mm = BALL_STATIC_POS_TARGET_MM;
        ball_balance_set_reference(
            BALL_STATIC_POS_TARGET_MM, 0.0f);
        if (position_mm > positive_peak_mm)
        {
            positive_peak_mm = position_mm;
        }
        if (target_is_stable(
                position_mm,
                velocity_mm_s,
                BALL_STATIC_POS_TARGET_MM) != 0U)
        {
            ball_static_state = BALL_STATIC_HOLD_POS;
            settle_start_ms = now_ms;
        }
        break;

    case BALL_STATIC_HOLD_POS:
        ball_static_target_mm = BALL_STATIC_POS_TARGET_MM;
        ball_balance_set_reference(
            BALL_STATIC_POS_TARGET_MM, 0.0f);
        if (position_mm > positive_peak_mm)
        {
            positive_peak_mm = position_mm;
        }
        if (target_is_stable(
                position_mm,
                velocity_mm_s,
                BALL_STATIC_POS_TARGET_MM) == 0U)
        {
            ball_static_state = BALL_STATIC_MOVE_POS;
            settle_start_ms = 0U;
        }
        else if ((uint32_t)(now_ms - settle_start_ms) >=
                 BALL_STATIC_POS_SETTLE_TIME_MS)
        {
            endpoint_error_mm = absolute_float(
                positive_peak_mm - BALL_STATIC_POS_TARGET_MM);
            ball_static_positive_max_error_mm =
                endpoint_error_mm;
            ball_static_target_mm =
                BALL_STATIC_NEG_TARGET_MM;
            ball_static_state = BALL_STATIC_MOVE_NEG;
            settle_start_ms = 0U;
            ball_balance_set_reference(
                BALL_STATIC_NEG_TARGET_MM, 0.0f);
        }
        break;

    case BALL_STATIC_MOVE_NEG:
        ball_static_target_mm = BALL_STATIC_NEG_TARGET_MM;
        ball_balance_set_reference(
            BALL_STATIC_NEG_TARGET_MM, 0.0f);
        if (target_is_stable(
                position_mm,
                velocity_mm_s,
                BALL_STATIC_NEG_TARGET_MM) != 0U)
        {
            ball_static_state = BALL_STATIC_HOLD_NEG;
            settle_start_ms = now_ms;
        }
        break;

    case BALL_STATIC_HOLD_NEG:
        ball_static_target_mm = BALL_STATIC_NEG_TARGET_MM;
        ball_balance_set_reference(
            BALL_STATIC_NEG_TARGET_MM, 0.0f);
        if (target_is_stable(
                position_mm,
                velocity_mm_s,
                BALL_STATIC_NEG_TARGET_MM) == 0U)
        {
            ball_static_state = BALL_STATIC_MOVE_NEG;
            settle_start_ms = 0U;
        }
        else if ((uint32_t)(now_ms - settle_start_ms) >=
                 BALL_STATIC_NEG_SETTLE_TIME_MS)
        {
            ball_static_state = BALL_STATIC_DONE;
            settle_start_ms = 0U;
        }
        break;

    case BALL_STATIC_DONE:
        ball_static_target_mm = BALL_STATIC_NEG_TARGET_MM;
        ball_balance_set_reference(
            BALL_STATIC_NEG_TARGET_MM, 0.0f);
        break;

    default:
        set_fault(
            BALL_STATIC_FAULT_TOTAL_TIMEOUT, now_ms);
        break;
    }
}

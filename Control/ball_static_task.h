#ifndef BALL_STATIC_TASK_H
#define BALL_STATIC_TASK_H

#include <stdint.h>

typedef enum
{
    BALL_STATIC_READY = 0,
    BALL_STATIC_MOVE_POS,
    BALL_STATIC_HOLD_POS,
    BALL_STATIC_MOVE_NEG,
    BALL_STATIC_HOLD_NEG,
    BALL_STATIC_PID_HOLD,
    BALL_STATIC_DONE,
    BALL_STATIC_FAULT
} ball_static_state_t;

typedef enum
{
    BALL_STATIC_FAULT_NONE = 0,
    BALL_STATIC_FAULT_VISION,
    BALL_STATIC_FAULT_START_POSITION,
    BALL_STATIC_FAULT_POS_TIMEOUT,
    BALL_STATIC_FAULT_TOTAL_TIMEOUT,
    BALL_STATIC_FAULT_OUT_OF_RANGE,
    BALL_STATIC_FAULT_ENDPOINT_ERROR
} ball_static_fault_t;

extern volatile ball_static_state_t ball_static_state;
extern volatile ball_static_fault_t ball_static_fault;
extern volatile uint32_t ball_static_elapsed_ms;
extern volatile float ball_static_positive_max_error_mm;
extern volatile float ball_static_negative_max_error_mm;
extern volatile uint8_t ball_static_ready;
extern volatile uint16_t ball_static_up_pulse_us;
extern volatile uint16_t ball_static_down_pulse_us;
extern volatile uint16_t ball_static_hold_delta_us;
extern volatile uint16_t ball_static_command_pulse_us;
extern volatile float ball_static_target_mm;
extern volatile float ball_static_positive_switch_mm;
extern volatile float ball_static_negative_switch_mm;
extern volatile float ball_static_hold_deadband_mm;
extern volatile float ball_static_velocity_threshold_mm_s;

void ball_static_task_init(void);
void ball_static_task_reset(void);
uint8_t ball_static_task_start(void);
void ball_static_task_stop(void);
void ball_static_task_update(void);
void ball_static_task_service(void);
uint8_t ball_static_task_controller_enabled(void);
uint8_t ball_static_task_is_running(void);
uint8_t ball_static_set_open_loop_config(
    uint16_t up_pulse_us,
    uint16_t down_pulse_us,
    float positive_switch_mm,
    float negative_switch_mm,
    uint16_t hold_delta_us,
    float hold_deadband_mm,
    float velocity_threshold_mm_s);
void ball_static_reset_open_loop_config(void);

#endif

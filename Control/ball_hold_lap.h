#ifndef BALL_HOLD_LAP_H
#define BALL_HOLD_LAP_H

#include <stdint.h>

#define BALL_HOLD_LAP_POST_DISTANCE_M (1.00f)
#define BALL_HOLD_LAP_MAX_ERROR_MM (10.0f)
#define BALL_HOLD_LAP_VISION_TIMEOUT_MS (200UL)
#define BALL_HOLD_LAP_TIME_LIMIT_MS (30000UL)

typedef enum
{
    BALL_HOLD_LAP_READY = 0,
    BALL_HOLD_LAP_CAPTURING,
    BALL_HOLD_LAP_RUNNING,
    BALL_HOLD_LAP_POST_LAP,
    BALL_HOLD_LAP_BRAKING,
    BALL_HOLD_LAP_DONE,
    BALL_HOLD_LAP_ABORTED,
    BALL_HOLD_LAP_FAULT
} ball_hold_lap_state_t;

extern volatile ball_hold_lap_state_t ball_hold_lap_state;
extern volatile float ball_hold_lap_target_mm;
extern volatile float ball_hold_lap_current_mm;
extern volatile float ball_hold_lap_error_mm;
extern volatile float ball_hold_lap_max_abs_error_mm;
extern volatile float ball_hold_lap_capture_mean_mm;
extern volatile float ball_hold_lap_capture_span_mm;
extern volatile uint16_t ball_hold_lap_capture_frames;
extern volatile uint32_t ball_hold_lap_capture_elapsed_ms;
extern volatile uint32_t ball_hold_lap_time_ms;
extern volatile uint8_t ball_hold_lap_current_valid;
extern volatile uint8_t ball_hold_lap_time_pass;
extern volatile uint8_t ball_hold_lap_position_pass;
extern volatile uint8_t ball_hold_lap_fault;

void ball_hold_lap_init(void);
void ball_hold_lap_reset(void);
void ball_hold_lap_start(void);
void ball_hold_lap_stop(void);
void ball_hold_lap_update(void);
uint8_t ball_hold_lap_controller_enabled(void);

#endif

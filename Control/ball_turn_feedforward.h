#ifndef BALL_TURN_FEEDFORWARD_H
#define BALL_TURN_FEEDFORWARD_H

#include <stdint.h>

#define BALL_TURN_FF_NOMINAL_LATERAL_COMMAND_MPS2 (0.1327f)
#define BALL_TURN_FF_HOLD_US (30.0f)
#define BALL_TURN_FF_ENTRY_EXTRA_US (20.0f)
#define BALL_TURN_FF_ENTRY_TICKS (30U)
#define BALL_TURN_FF_RELEASE_TICKS (20U)
#define BALL_TURN_FF_SCALE_MAX (2.0f)
#define BALL_TURN_FF_LIMIT_US (100.0f)
#define BALL_TURN_FF_SPEED_LIMIT_MPS (1.0f)
#define BALL_TURN_FF_OMEGA_LIMIT_RAD_S (5.0f)

typedef struct
{
    float output_us;
    float release_start_us;
    uint16_t entry_ticks_remaining;
    uint16_t release_ticks_remaining;
    uint8_t previous_arc_active;
    uint8_t active;
    uint8_t entry_active;
} ball_turn_feedforward_t;

void ball_turn_feedforward_reset(
    ball_turn_feedforward_t *state);

uint8_t ball_turn_feedforward_update(
    ball_turn_feedforward_t *state,
    uint8_t arc_active,
    float commanded_speed_mps,
    float commanded_omega_rad_s);

#endif

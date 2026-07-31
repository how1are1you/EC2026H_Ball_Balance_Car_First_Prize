#ifndef STARTUP_MOTION_GATE_H
#define STARTUP_MOTION_GATE_H

#include <stdint.h>

#define STARTUP_MOTION_GATE_SPEED_THRESHOLD_MPS (0.02f)
#define STARTUP_MOTION_GATE_CONFIRM_TICKS (2U)

typedef struct
{
    uint8_t consecutive_motion_ticks;
    uint8_t motion_detected;
} startup_motion_gate_t;

void startup_motion_gate_reset(startup_motion_gate_t *gate);
uint8_t startup_motion_gate_update(
    startup_motion_gate_t *gate,
    float left_speed_mps,
    float right_speed_mps);

#endif

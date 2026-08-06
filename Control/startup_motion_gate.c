#include "startup_motion_gate.h"

void startup_motion_gate_reset(startup_motion_gate_t *gate)
{
    gate->consecutive_motion_ticks = 0U;
    gate->motion_detected = 0U;
}

uint8_t startup_motion_gate_update(
    startup_motion_gate_t *gate,
    float left_speed_mps,
    float right_speed_mps)
{
    float longitudinal_speed_mps;

    if (gate->motion_detected != 0U)
    {
        return 1U;
    }

    longitudinal_speed_mps =
        0.5f * (left_speed_mps + right_speed_mps);
    if (longitudinal_speed_mps >
        STARTUP_MOTION_GATE_SPEED_THRESHOLD_MPS)
    {
        gate->consecutive_motion_ticks++;
        if (gate->consecutive_motion_ticks >=
            STARTUP_MOTION_GATE_CONFIRM_TICKS)
        {
            gate->motion_detected = 1U;
        }
    }
    else
    {
        gate->consecutive_motion_ticks = 0U;
    }

    return gate->motion_detected;
}

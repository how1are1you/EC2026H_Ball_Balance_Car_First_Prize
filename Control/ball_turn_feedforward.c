#include "ball_turn_feedforward.h"

#include <stddef.h>

static uint8_t finite_float(float value)
{
    return
        (value == value &&
         value <= 3.402823466e+38F &&
         value >= -3.402823466e+38F)
            ? 1U : 0U;
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

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

void ball_turn_feedforward_reset(
    ball_turn_feedforward_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->output_us = 0.0f;
    state->release_start_us = 0.0f;
    state->entry_ticks_remaining = 0U;
    state->release_ticks_remaining = 0U;
    state->previous_arc_active = 0U;
    state->active = 0U;
    state->entry_active = 0U;
}

uint8_t ball_turn_feedforward_update(
    ball_turn_feedforward_t *state,
    uint8_t arc_active,
    float commanded_speed_mps,
    float commanded_omega_rad_s)
{
    float lateral_command_mps2;
    float scale;
    float direction;
    float entry_decay;
    float requested_output_us;

    if (state == NULL)
    {
        return 0U;
    }
    if (finite_float(commanded_speed_mps) == 0U ||
        finite_float(commanded_omega_rad_s) == 0U ||
        absolute_float(commanded_speed_mps) >
            BALL_TURN_FF_SPEED_LIMIT_MPS ||
        absolute_float(commanded_omega_rad_s) >
            BALL_TURN_FF_OMEGA_LIMIT_RAD_S)
    {
        ball_turn_feedforward_reset(state);
        return 0U;
    }

    arc_active = (arc_active != 0U) ? 1U : 0U;
    if (arc_active != 0U)
    {
        if (state->previous_arc_active == 0U)
        {
            state->entry_ticks_remaining =
                BALL_TURN_FF_ENTRY_TICKS;
            state->release_ticks_remaining = 0U;
        }

        lateral_command_mps2 = absolute_float(
            commanded_speed_mps * commanded_omega_rad_s);
        scale = clamp_float(
            lateral_command_mps2 /
                BALL_TURN_FF_NOMINAL_LATERAL_COMMAND_MPS2,
            0.0f,
            BALL_TURN_FF_SCALE_MAX);
        direction =
            (commanded_omega_rad_s < 0.0f) ? -1.0f :
            (commanded_omega_rad_s > 0.0f) ? 1.0f : 0.0f;
        entry_decay =
            (float)state->entry_ticks_remaining /
            (float)BALL_TURN_FF_ENTRY_TICKS;
        requested_output_us =
            direction * scale *
            (BALL_TURN_FF_HOLD_US +
             BALL_TURN_FF_ENTRY_EXTRA_US * entry_decay);
        state->output_us = clamp_float(
            requested_output_us,
            -BALL_TURN_FF_LIMIT_US,
            BALL_TURN_FF_LIMIT_US);
        state->entry_active =
            (state->entry_ticks_remaining != 0U) ? 1U : 0U;
        if (state->entry_ticks_remaining != 0U)
        {
            state->entry_ticks_remaining--;
        }
    }
    else
    {
        state->entry_ticks_remaining = 0U;
        state->entry_active = 0U;
        if (state->previous_arc_active != 0U)
        {
            state->release_start_us = state->output_us;
            state->release_ticks_remaining =
                BALL_TURN_FF_RELEASE_TICKS;
        }
        if (state->release_ticks_remaining != 0U)
        {
            state->release_ticks_remaining--;
            state->output_us =
                state->release_start_us *
                (float)state->release_ticks_remaining /
                (float)BALL_TURN_FF_RELEASE_TICKS;
        }
        else
        {
            state->output_us = 0.0f;
        }
    }

    state->active =
        (state->output_us < -0.001f ||
         state->output_us > 0.001f) ? 1U : 0U;
    state->previous_arc_active = arc_active;
    return 1U;
}

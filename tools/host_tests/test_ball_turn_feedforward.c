#include "ball_turn_feedforward.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.01f);
}

int main(void)
{
    ball_turn_feedforward_t state;
    uint16_t tick;

    ball_turn_feedforward_reset(&state);
    assert_close(state.output_us, 0.0f);
    assert(state.active == 0U);
    assert(state.entry_active == 0U);

    assert(ball_turn_feedforward_update(
               &state, 0U, 0.25f, 0.0f) == 1U);
    assert_close(state.output_us, 0.0f);

    assert(ball_turn_feedforward_update(
               &state, 1U, 0.1327f, -1.0f) == 1U);
    assert_close(state.output_us, -50.0f);
    assert(state.active == 1U);
    assert(state.entry_active == 1U);

    for (tick = 0U;
         tick < BALL_TURN_FF_ENTRY_TICKS;
         tick++)
    {
        assert(ball_turn_feedforward_update(
                   &state, 1U, 0.1327f, -1.0f) == 1U);
    }
    assert_close(state.output_us, -30.0f);
    assert(state.entry_active == 0U);

    assert(ball_turn_feedforward_update(
               &state, 0U, 0.25f, 0.0f) == 1U);
    assert(state.output_us < 0.0f);
    assert(state.output_us > -30.0f);

    for (tick = 1U;
         tick < BALL_TURN_FF_RELEASE_TICKS;
         tick++)
    {
        assert(ball_turn_feedforward_update(
                   &state, 0U, 0.25f, 0.0f) == 1U);
    }
    assert_close(state.output_us, 0.0f);
    assert(state.active == 0U);

    ball_turn_feedforward_reset(&state);
    assert(ball_turn_feedforward_update(
               &state, 1U, 0.1327f, 1.0f) == 1U);
    assert_close(state.output_us, 50.0f);

    ball_turn_feedforward_reset(&state);
    assert(ball_turn_feedforward_update(
               &state, 1U, 1.0f, -5.0f) == 1U);
    assert_close(state.output_us, -100.0f);

    assert(ball_turn_feedforward_update(
               &state, 1U, NAN, -1.0f) == 0U);
    assert_close(state.output_us, 0.0f);
    assert(state.active == 0U);
    assert(ball_turn_feedforward_update(
               NULL, 1U, 0.25f, -0.5f) == 0U);

    return 0;
}

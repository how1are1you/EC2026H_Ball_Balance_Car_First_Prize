#include "ball_position_feedforward.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    assert_close(ball_position_feedforward_us(-75.0f), 1320.0f);
    assert_close(ball_position_feedforward_us(-50.0f), 1320.0f);
    assert_close(ball_position_feedforward_us(-25.0f), 1320.0f);
    assert_close(ball_position_feedforward_us(0.0f), 1320.0f);
    assert_close(ball_position_feedforward_us(12.5f), 1355.0f);
    assert_close(ball_position_feedforward_us(25.0f), 1390.0f);
    assert_close(ball_position_feedforward_us(37.5f), 1410.0f);
    assert_close(ball_position_feedforward_us(50.0f), 1430.0f);
    assert_close(ball_position_feedforward_us(75.0f), 1430.0f);

    return 0;
}

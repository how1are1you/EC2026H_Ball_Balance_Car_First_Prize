#include "ball_position_feedforward.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    assert_close(ball_position_feedforward_us(-75.0f), 1295.0f);
    assert_close(ball_position_feedforward_us(-50.0f), 1295.0f);
    assert_close(ball_position_feedforward_us(-37.5f), 1302.5f);
    assert_close(ball_position_feedforward_us(-25.0f), 1310.0f);
    assert_close(ball_position_feedforward_us(-12.5f), 1342.5f);
    assert_close(ball_position_feedforward_us(0.0f), 1375.0f);
    assert_close(ball_position_feedforward_us(12.5f), 1392.5f);
    assert_close(ball_position_feedforward_us(25.0f), 1410.0f);
    assert_close(ball_position_feedforward_us(37.5f), 1432.5f);
    assert_close(ball_position_feedforward_us(50.0f), 1455.0f);
    assert_close(ball_position_feedforward_us(75.0f), 1455.0f);

    return 0;
}

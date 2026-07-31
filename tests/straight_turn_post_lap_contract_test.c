#include "straight_turn_test.h"

static void (*start_with_post_lap)(float, float, float) =
    StraightTurnTest_StartWithPostLap;

int main(void)
{
    return (STRAIGHT_TURN_POST_LAP ==
            STRAIGHT_TURN_BRAKING) ||
           (start_with_post_lap == 0);
}

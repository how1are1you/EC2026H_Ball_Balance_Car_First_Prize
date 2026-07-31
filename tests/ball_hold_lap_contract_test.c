#include "ball_hold_lap.h"

static void (*start_task)(void) = ball_hold_lap_start;
static void (*update_task)(void) = ball_hold_lap_update;

int main(void)
{
    return (BALL_HOLD_LAP_READY ==
            BALL_HOLD_LAP_DONE) ||
           (start_task == 0) ||
           (update_task == 0);
}

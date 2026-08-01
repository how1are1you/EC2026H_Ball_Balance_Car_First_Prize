#include "ball_position_feedforward.h"

#define BALL_FEEDFORWARD_NEG_MAX_POSITION_MM (-50.0f)
#define BALL_FEEDFORWARD_NEG_MID_POSITION_MM (-25.0f)
#define BALL_FEEDFORWARD_ZERO_POSITION_MM (0.0f)
#define BALL_FEEDFORWARD_POS_MID_POSITION_MM (25.0f)
#define BALL_FEEDFORWARD_POS_MAX_POSITION_MM (50.0f)

#define BALL_FEEDFORWARD_NEG_MAX_PULSE_US (1295.0f)
#define BALL_FEEDFORWARD_NEG_MID_PULSE_US (1310.0f)
#define BALL_FEEDFORWARD_ZERO_PULSE_US (1375.0f)
#define BALL_FEEDFORWARD_POS_MID_PULSE_US (1410.0f)
#define BALL_FEEDFORWARD_POS_MAX_PULSE_US (1455.0f)

static float interpolate(
    float position_mm,
    float lower_position_mm,
    float upper_position_mm,
    float lower_pulse_us,
    float upper_pulse_us)
{
    return lower_pulse_us +
           (position_mm - lower_position_mm) *
               (upper_pulse_us - lower_pulse_us) /
               (upper_position_mm - lower_position_mm);
}

float ball_position_feedforward_us(float position_mm)
{
    if (position_mm <= BALL_FEEDFORWARD_NEG_MAX_POSITION_MM)
    {
        return BALL_FEEDFORWARD_NEG_MAX_PULSE_US;
    }
    if (position_mm < BALL_FEEDFORWARD_NEG_MID_POSITION_MM)
    {
        return interpolate(
            position_mm,
            BALL_FEEDFORWARD_NEG_MAX_POSITION_MM,
            BALL_FEEDFORWARD_NEG_MID_POSITION_MM,
            BALL_FEEDFORWARD_NEG_MAX_PULSE_US,
            BALL_FEEDFORWARD_NEG_MID_PULSE_US);
    }
    if (position_mm < BALL_FEEDFORWARD_ZERO_POSITION_MM)
    {
        return interpolate(
            position_mm,
            BALL_FEEDFORWARD_NEG_MID_POSITION_MM,
            BALL_FEEDFORWARD_ZERO_POSITION_MM,
            BALL_FEEDFORWARD_NEG_MID_PULSE_US,
            BALL_FEEDFORWARD_ZERO_PULSE_US);
    }
    if (position_mm < BALL_FEEDFORWARD_POS_MID_POSITION_MM)
    {
        return interpolate(
            position_mm,
            BALL_FEEDFORWARD_ZERO_POSITION_MM,
            BALL_FEEDFORWARD_POS_MID_POSITION_MM,
            BALL_FEEDFORWARD_ZERO_PULSE_US,
            BALL_FEEDFORWARD_POS_MID_PULSE_US);
    }
    if (position_mm < BALL_FEEDFORWARD_POS_MAX_POSITION_MM)
    {
        return interpolate(
            position_mm,
            BALL_FEEDFORWARD_POS_MID_POSITION_MM,
            BALL_FEEDFORWARD_POS_MAX_POSITION_MM,
            BALL_FEEDFORWARD_POS_MID_PULSE_US,
            BALL_FEEDFORWARD_POS_MAX_PULSE_US);
    }
    return BALL_FEEDFORWARD_POS_MAX_PULSE_US;
}

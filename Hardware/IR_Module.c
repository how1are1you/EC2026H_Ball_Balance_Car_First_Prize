#include "IR_Module.h"
#include "control.h"

uint8_t LF04_DH1_State;
uint8_t LF04_DH2_State;
uint8_t LF04_DH3_State;
uint8_t LF04_DH4_State;
uint8_t LF04_State;

uint8_t LF04_BlackActiveHigh = 1U;

float LF04_BaseSpeed = 0.12f;
float LF04_LostLineSpeed = 0.06f;
float LF04_Turn90Angle = 0.40f;
float LF04_TurnMaxAngle = 0.28f;
float LF04_TurnMinAngle = 0.12f;
float LF04_SteeringSign = 1.0f;
float LF04_Acceleration = 0.20f;

typedef enum
{
    LF04_STATE_CROSS = 0x0U,
    LF04_STATE_LEFT_90_A = 0x1U,
    LF04_STATE_LEFT_90_B = 0x3U,
    LF04_STATE_RIGHT_90_A = 0x8U,
    LF04_STATE_RIGHT_90_B = 0xCU,
    LF04_STATE_LEFT_BIG = 0x7U,
    LF04_STATE_RIGHT_BIG = 0xEU,
    LF04_STATE_LEFT_SMALL = 0xBU,
    LF04_STATE_RIGHT_SMALL = 0xDU,
    LF04_STATE_STRAIGHT = 0x9U,
    LF04_STATE_LOST = 0xFU
} LF04_State_t;

static uint8_t LF04_LastState = LF04_STATE_STRAIGHT;
static float LF04_RampedSpeed;

static uint8_t LF04_IsBlack(uint32_t pinState)
{
    uint8_t isHigh = (pinState != 0U) ? 1U : 0U;

    return (LF04_BlackActiveHigh != 0U) ? isHigh : (uint8_t)(isHigh == 0U);
}

void IR_Ackermann_LineTrack(void)
{
    float targetSpeed = LF04_BaseSpeed;
    float steeringAngle = 0.0f;
    const float speedStep = LF04_Acceleration * 0.005f;

    LF04_DH1_State = LF04_IsBlack(DL_GPIO_readPins(LF04_DH1_PORT, LF04_DH1_PIN));
    LF04_DH2_State = LF04_IsBlack(DL_GPIO_readPins(LF04_DH2_PORT, LF04_DH2_PIN));
    LF04_DH3_State = LF04_IsBlack(DL_GPIO_readPins(LF04_DH3_PORT, LF04_DH3_PIN));
    LF04_DH4_State = LF04_IsBlack(DL_GPIO_readPins(LF04_DH4_PORT, LF04_DH4_PIN));

    LF04_State = (uint8_t)((LF04_DH1_State << 3U) |
                            (LF04_DH2_State << 2U) |
                            (LF04_DH3_State << 1U) |
                            LF04_DH4_State);

    switch (LF04_State)
    {
        case LF04_STATE_LEFT_90_A:
        case LF04_STATE_LEFT_90_B:
            steeringAngle = LF04_Turn90Angle;
            targetSpeed = LF04_LostLineSpeed;
            break;

        case LF04_STATE_RIGHT_90_A:
        case LF04_STATE_RIGHT_90_B:
            steeringAngle = -LF04_Turn90Angle;
            targetSpeed = LF04_LostLineSpeed;
            break;

        case LF04_STATE_LEFT_BIG:
            steeringAngle = LF04_TurnMaxAngle;
            break;

        case LF04_STATE_RIGHT_BIG:
            steeringAngle = -LF04_TurnMaxAngle;
            break;

        case LF04_STATE_LEFT_SMALL:
            steeringAngle = LF04_TurnMinAngle;
            break;

        case LF04_STATE_RIGHT_SMALL:
            steeringAngle = -LF04_TurnMinAngle;
            break;

        case LF04_STATE_LOST:
            targetSpeed = LF04_LostLineSpeed;
            if (LF04_LastState == LF04_STATE_LEFT_90_A ||
                LF04_LastState == LF04_STATE_LEFT_90_B ||
                LF04_LastState == LF04_STATE_LEFT_BIG ||
                LF04_LastState == LF04_STATE_LEFT_SMALL)
            {
                steeringAngle = LF04_TurnMaxAngle;
            }
            else if (LF04_LastState == LF04_STATE_RIGHT_90_A ||
                     LF04_LastState == LF04_STATE_RIGHT_90_B ||
                     LF04_LastState == LF04_STATE_RIGHT_BIG ||
                     LF04_LastState == LF04_STATE_RIGHT_SMALL)
            {
                steeringAngle = -LF04_TurnMaxAngle;
            }
            break;

        case LF04_STATE_CROSS:
        case LF04_STATE_STRAIGHT:
        default:
            break;
    }

    if (LF04_State != LF04_STATE_LOST)
    {
        LF04_LastState = LF04_State;
    }

    if (targetSpeed > LF04_RampedSpeed + speedStep)
    {
        LF04_RampedSpeed += speedStep;
    }
    else if (targetSpeed < LF04_RampedSpeed - speedStep)
    {
        LF04_RampedSpeed -= speedStep;
    }
    else
    {
        LF04_RampedSpeed = targetSpeed;
    }

    Get_Target_Encoder(LF04_RampedSpeed, LF04_SteeringSign * steeringAngle);
}

void IR_Ackermann_LineTrack_Reset(void)
{
    LF04_LastState = LF04_STATE_STRAIGHT;
    LF04_RampedSpeed = 0.0f;
}

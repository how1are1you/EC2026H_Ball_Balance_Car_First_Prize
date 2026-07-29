#include "IR_Module.h"
#include "control.h"
#include "imu/imu.h"

#include <math.h>

uint8_t LF04_DH1_State;
uint8_t LF04_DH2_State;
uint8_t LF04_DH3_State;
uint8_t LF04_DH4_State;
uint8_t LF04_State;

uint8_t LF04_BlackActiveHigh = 1U;

float LF04_BaseSpeed = 0.24f;
float LF04_LostLineSpeed = 0.10f;
float LF04_SteeringSign = -1.0f;
float LF04_Acceleration = 0.30f;
float LF04_Deceleration = 0.35f;
float LF04_SteeringKp = 0.45f;
float LF04_SteeringKi = 0.0f;
float LF04_SteeringKd = 0.0008f;
float LF04_MaxFeedbackOmega = 0.65f;
/*
 * Reduce this value when the vehicle cuts inside the black semicircle;
 * increase it when the turn radius is too large. It does not affect the
 * encoder-based lap stopping distance.
 */
float LF04_CurveFeedforwardGain = 0.70f;
/*
 * 0.34 m is the currently measured sensor-to-rear-edge distance and is
 * only a first estimate of the sensor-to-drive-axle distance. Measure the
 * latter directly, then calibrate LF04_LapTargetDistance on the real track.
 */
float LF04_SensorToDriveAxle = 0.34f;
float LF04_LapTargetDistance = 5.3033f;
float LF04_FinishYawMinimum = 270.0f;

#define LF04_CONTROL_PERIOD_S (0.005f)
#define LF04_CURVE_RADIUS_M (0.50f)
#define LF04_STRAIGHT_LENGTH_M (1.5000f)
#define LF04_PI (3.1415926f)
#define LF04_CURVATURE_BLEND_M (0.080f)
#define LF04_STEERING_INTEGRAL_LIMIT (1.00f)
#define LF04_MAX_COMMAND_OMEGA (1.20f)
#define LF04_MAX_DISTANCE_STEP_M (0.005f)
#define LF04_FINISH_ARM_MARGIN_M (0.50f)
#define LF04_FINISH_YAW_MAXIMUM_DEG (450.0f)
#define LF04_LINE_LOST_TIMEOUT_TICKS (160U)
#define LF04_IMU_STALE_TIMEOUT_TICKS (100U)
#define LF04_RUN_TIMEOUT_TICKS (7000U)
#define LF04_SETTLE_SPEED_MPS (0.015f)
#define LF04_SETTLE_REQUIRED_TICKS (40U)
#define LF04_SETTLE_TIMEOUT_TICKS (200U)

volatile LF04_LapState_t LF04_LapState = LF04_LAP_IDLE;
volatile uint8_t LF04_LapFault = LF04_LAP_FAULT_NONE;
volatile uint8_t LF04_ImuUsed;
volatile float LF04_LapDistanceM;
volatile float LF04_LapYawDeg;
volatile float LF04_SteeringError;
volatile float LF04_CommandSpeed;
volatile uint32_t LF04_LapElapsedMs;

static float LF04_LastNonzeroError = 1.0f;
static float LF04_LastError;
static float LF04_ErrorDerivative;
static float LF04_ErrorIntegral;
static float LF04_ImuStartYaw;
static uint32_t LF04_ImuLastSampleCount;
static uint16_t LF04_ImuStaleTicks;
static uint16_t LF04_LineLostTicks;
static uint16_t LF04_ElapsedTicks;
static uint16_t LF04_SettleTicks;
static uint16_t LF04_SettleElapsedTicks;

static uint8_t LF04_IsBlack(uint32_t pinState)
{
    uint8_t isHigh = (pinState != 0U) ? 1U : 0U;

    return (LF04_BlackActiveHigh != 0U) ? isHigh : (uint8_t)(isHigh == 0U);
}

uint8_t IR_ReadLineErrorFour(float *error)
{
    float weighted_sum;
    uint8_t active_count;

    LF04_DH1_State = LF04_IsBlack(
        DL_GPIO_readPins(LF04_DH1_PORT, LF04_DH1_PIN));
    LF04_DH2_State = LF04_IsBlack(
        DL_GPIO_readPins(LF04_DH2_PORT, LF04_DH2_PIN));
    LF04_DH3_State = LF04_IsBlack(
        DL_GPIO_readPins(LF04_DH3_PORT, LF04_DH3_PIN));
    LF04_DH4_State = LF04_IsBlack(
        DL_GPIO_readPins(LF04_DH4_PORT, LF04_DH4_PIN));
    LF04_State =
        (uint8_t)((LF04_DH1_State << 3U) |
                  (LF04_DH2_State << 2U) |
                  (LF04_DH3_State << 1U) |
                  LF04_DH4_State);

    active_count =
        LF04_DH1_State + LF04_DH2_State +
        LF04_DH3_State + LF04_DH4_State;
    if (active_count == 0U)
    {
        return 0U;
    }

    /*
     * Keep the same steering sign convention as the existing two-channel
     * controller: DH1/DH2 are on one side and DH3/DH4 on the other.
     */
    weighted_sum =
        3.0f * (float)LF04_DH1_State +
        1.8f * (float)LF04_DH2_State -
        1.8f * (float)LF04_DH3_State -
        3.0f * (float)LF04_DH4_State;
    *error = LF04_SteeringSign *
             weighted_sum / (float)active_count;
    return 1U;
}

static float LF04_LimitFloat(float value, float minimum, float maximum)
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

static float LF04_SmoothStep(float value)
{
    value = LF04_LimitFloat(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

static float LF04_DriveAxleCurveRadius(void)
{
    float radiusSquared =
        LF04_CURVE_RADIUS_M * LF04_CURVE_RADIUS_M -
        LF04_SensorToDriveAxle * LF04_SensorToDriveAxle;

    if (radiusSquared < 0.01f)
    {
        radiusSquared = 0.01f;
    }
    return sqrtf(radiusSquared);
}

static float LF04_CurveBlend(
    float distance,
    float curveStart,
    float curveEnd)
{
    float startBlend = LF04_SmoothStep(
        (distance - (curveStart - LF04_CURVATURE_BLEND_M)) /
        (2.0f * LF04_CURVATURE_BLEND_M));
    float endBlend = 1.0f - LF04_SmoothStep(
                                (distance - (curveEnd - LF04_CURVATURE_BLEND_M)) /
                                (2.0f * LF04_CURVATURE_BLEND_M));

    return startBlend * endBlend;
}

static float LF04_CourseCurvature(float distance)
{
    float axleCurveRadius = LF04_DriveAxleCurveRadius();
    float firstCurveEnd =
        LF04_STRAIGHT_LENGTH_M + LF04_PI * axleCurveRadius;
    float secondStraightEnd =
        firstCurveEnd + LF04_STRAIGHT_LENGTH_M;
    float curveBlend =
        LF04_CurveBlend(distance,
                        LF04_STRAIGHT_LENGTH_M,
                        firstCurveEnd) +
        LF04_CurveBlend(distance,
                        secondStraightEnd,
                        LF04_LapTargetDistance);

    return -LF04_CurveFeedforwardGain *
           LF04_LimitFloat(curveBlend, 0.0f, 1.0f) /
           axleCurveRadius;
}

static void LF04_UpdateCourseState(void)
{
    float firstCurveEnd =
        LF04_STRAIGHT_LENGTH_M +
        LF04_PI * LF04_DriveAxleCurveRadius();
    float secondStraightEnd =
        firstCurveEnd + LF04_STRAIGHT_LENGTH_M;

    if (LF04_LapDistanceM < LF04_STRAIGHT_LENGTH_M)
    {
        LF04_LapState = LF04_LAP_STRAIGHT_1;
    }
    else if (LF04_LapDistanceM < firstCurveEnd)
    {
        LF04_LapState = LF04_LAP_CURVE_1;
    }
    else if (LF04_LapDistanceM < secondStraightEnd)
    {
        LF04_LapState = LF04_LAP_STRAIGHT_2;
    }
    else
    {
        LF04_LapState = LF04_LAP_CURVE_2;
    }
}

static void LF04_StopWithFault(uint8_t fault)
{
    LF04_LapFault = fault;
    LF04_LapState = LF04_LAP_FAULT;
    LF04_CommandSpeed = 0.0f;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    Flag_Stop = 1U;
}

static void LF04_UpdateImu(void)
{
    imu_sample_t sample;

    imu_get_snapshot(&sample);
    if (LF04_ImuUsed == 0U)
    {
        return;
    }

    if (sample.status == IMU_STATUS_READY &&
        sample.valid != 0U &&
        sample.sample_count != LF04_ImuLastSampleCount)
    {
        LF04_ImuLastSampleCount = sample.sample_count;
        LF04_ImuStaleTicks = 0U;
        LF04_LapYawDeg =
            sample.yaw_continuous_deg - LF04_ImuStartYaw;
    }
    else if (LF04_ImuStaleTicks < LF04_IMU_STALE_TIMEOUT_TICKS)
    {
        LF04_ImuStaleTicks++;
    }
    else
    {
        /*
         * Line tracking remains available if the DMP stream is lost.
         * Keep running on the encoder distance, but latch a visible fault.
         */
        LF04_ImuUsed = 0U;
        LF04_LapFault = LF04_LAP_FAULT_IMU_STALE;
    }
}

static void LF04_StartRun(void)
{
    imu_sample_t sample;

    imu_get_snapshot(&sample);
    if (sample.status == IMU_STATUS_READY && sample.valid != 0U)
    {
        LF04_ImuUsed = 1U;
        LF04_ImuStartYaw = sample.yaw_continuous_deg;
        LF04_ImuLastSampleCount = sample.sample_count;
    }
    else
    {
        LF04_LapFault = LF04_LAP_FAULT_IMU_STALE;
    }
    LF04_LapState = LF04_LAP_STRAIGHT_1;
}

static float LF04_ReadSteeringError(void)
{
    float error;

    if (IR_ReadLineErrorFour(&error) != 0U)
    {
        LF04_LineLostTicks = 0U;
        if (fabsf(error) > 0.001f)
        {
            LF04_LastNonzeroError = error;
        }
        return error;
    }

    error = 1.5f * LF04_LastNonzeroError;
    if (LF04_LineLostTicks < LF04_LINE_LOST_TIMEOUT_TICKS)
    {
        LF04_LineLostTicks++;
    }
    return error;
}

void IR_Differential_OneLap(void)
{
    float distanceStep;
    float rawDerivative;
    float feedbackOmega;
    float feedforwardOmega;
    float commandOmega;
    float desiredSpeed;
    float remainingDistance;
    float brakingSpeed;
    float speedStep;
    uint8_t finishArmed;

    if (LF04_LapState == LF04_LAP_IDLE)
    {
        LF04_StartRun();
    }

    distanceStep =
        0.5f * (MotorA.Current_Encoder + MotorB.Current_Encoder) *
        LF04_CONTROL_PERIOD_S;
    LF04_LapDistanceM +=
        LF04_LimitFloat(distanceStep, 0.0f, LF04_MAX_DISTANCE_STEP_M);
    LF04_ElapsedTicks++;
    LF04_LapElapsedMs = (uint32_t)LF04_ElapsedTicks * 5U;
    LF04_UpdateImu();

    if (LF04_ElapsedTicks >= LF04_RUN_TIMEOUT_TICKS)
    {
        LF04_StopWithFault(LF04_LAP_FAULT_TIMEOUT);
        return;
    }

    if (LF04_LapState == LF04_LAP_SETTLING)
    {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        LF04_CommandSpeed = 0.0f;
        LF04_SettleElapsedTicks++;

        if (fabsf(MotorA.Current_Encoder) <= LF04_SETTLE_SPEED_MPS &&
            fabsf(MotorB.Current_Encoder) <= LF04_SETTLE_SPEED_MPS)
        {
            LF04_SettleTicks++;
            if (LF04_SettleTicks >= LF04_SETTLE_REQUIRED_TICKS)
            {
                LF04_LapState =
                    (LF04_LapFault == LF04_LAP_FAULT_NONE) ? LF04_LAP_DONE : LF04_LAP_FAULT;
                Flag_Stop = 1U;
            }
        }
        else
        {
            LF04_SettleTicks = 0U;
        }
        if (LF04_SettleElapsedTicks >= LF04_SETTLE_TIMEOUT_TICKS)
        {
            LF04_LapState =
                (LF04_LapFault == LF04_LAP_FAULT_NONE) ? LF04_LAP_DONE : LF04_LAP_FAULT;
            Flag_Stop = 1U;
        }
        return;
    }

    LF04_SteeringError = LF04_ReadSteeringError();
    if (LF04_LineLostTicks >= LF04_LINE_LOST_TIMEOUT_TICKS)
    {
        LF04_StopWithFault(LF04_LAP_FAULT_LINE_LOST);
        return;
    }

    rawDerivative =
        (LF04_SteeringError - LF04_LastError) /
        LF04_CONTROL_PERIOD_S;
    LF04_ErrorDerivative +=
        0.20f * (rawDerivative - LF04_ErrorDerivative);
    LF04_ErrorIntegral +=
        LF04_SteeringError * LF04_CONTROL_PERIOD_S;
    LF04_ErrorIntegral = LF04_LimitFloat(
        LF04_ErrorIntegral,
        -LF04_STEERING_INTEGRAL_LIMIT,
        LF04_STEERING_INTEGRAL_LIMIT);
    LF04_LastError = LF04_SteeringError;

    feedbackOmega =
        LF04_SteeringKp * LF04_SteeringError +
        LF04_SteeringKi * LF04_ErrorIntegral +
        LF04_SteeringKd * LF04_ErrorDerivative;
    feedbackOmega = LF04_LimitFloat(
        feedbackOmega,
        -LF04_MaxFeedbackOmega,
        LF04_MaxFeedbackOmega);

    finishArmed =
        (LF04_LapDistanceM >=
         LF04_LapTargetDistance - LF04_FINISH_ARM_MARGIN_M);

    desiredSpeed =
        (LF04_State == 0x00U) ? LF04_LostLineSpeed : LF04_BaseSpeed;
    remainingDistance =
        LF04_LapTargetDistance - LF04_LapDistanceM;
    if (finishArmed != 0U)
    {
        if (remainingDistance <= 0.0f)
        {
            if (LF04_ImuUsed != 0U &&
                (fabsf(LF04_LapYawDeg) <
                     LF04_FinishYawMinimum ||
                 fabsf(LF04_LapYawDeg) >
                     LF04_FINISH_YAW_MAXIMUM_DEG))
            {
                LF04_LapFault = LF04_LAP_FAULT_PROGRESS;
            }
            LF04_CommandSpeed = 0.0f;
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
            LF04_LapState = LF04_LAP_SETTLING;
            LF04_SettleTicks = 0U;
            LF04_SettleElapsedTicks = 0U;
            return;
        }

        brakingSpeed = sqrtf(
            2.0f * LF04_Deceleration * remainingDistance);
        if (brakingSpeed < desiredSpeed)
        {
            desiredSpeed = brakingSpeed;
            LF04_LapState = LF04_LAP_BRAKING;
        }
        else
        {
            LF04_UpdateCourseState();
        }
    }
    else
    {
        LF04_UpdateCourseState();
    }

    speedStep = ((desiredSpeed >= LF04_CommandSpeed) ? LF04_Acceleration : LF04_Deceleration) *
                LF04_CONTROL_PERIOD_S;
    if (desiredSpeed > LF04_CommandSpeed + speedStep)
    {
        LF04_CommandSpeed += speedStep;
    }
    else if (desiredSpeed < LF04_CommandSpeed - speedStep)
    {
        LF04_CommandSpeed -= speedStep;
    }
    else
    {
        LF04_CommandSpeed = desiredSpeed;
    }

    feedforwardOmega =
        LF04_CourseCurvature(LF04_LapDistanceM) *
        LF04_CommandSpeed;
    commandOmega = LF04_LimitFloat(
        feedforwardOmega + feedbackOmega,
        -LF04_MAX_COMMAND_OMEGA,
        LF04_MAX_COMMAND_OMEGA);

    MotorA.Target_Encoder =
        LF04_CommandSpeed -
        0.5f * DRIVE_WHEEL_SPACING * commandOmega;
    MotorB.Target_Encoder =
        LF04_CommandSpeed +
        0.5f * DRIVE_WHEEL_SPACING * commandOmega;
}

void IR_Differential_OneLap_Reset(void)
{
    LF04_DH1_State = 0U;
    LF04_DH2_State = 0U;
    LF04_DH3_State = 0U;
    LF04_DH4_State = 0U;
    LF04_State = 0U;
    LF04_LapState = LF04_LAP_IDLE;
    LF04_LapFault = LF04_LAP_FAULT_NONE;
    LF04_ImuUsed = 0U;
    LF04_LapDistanceM = 0.0f;
    LF04_LapYawDeg = 0.0f;
    LF04_SteeringError = 0.0f;
    LF04_CommandSpeed = 0.0f;
    LF04_LapElapsedMs = 0U;
    LF04_LastNonzeroError = -LF04_SteeringSign;
    LF04_LastError = 0.0f;
    LF04_ErrorDerivative = 0.0f;
    LF04_ErrorIntegral = 0.0f;
    LF04_ImuStartYaw = 0.0f;
    LF04_ImuLastSampleCount = 0U;
    LF04_ImuStaleTicks = 0U;
    LF04_LineLostTicks = 0U;
    LF04_ElapsedTicks = 0U;
    LF04_SettleTicks = 0U;
    LF04_SettleElapsedTicks = 0U;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
}

#include "straight_turn_test.h"

#include "control.h"
#include "IR_Module.h"
#include "imu/imu.h"
#include "turn_calibration.h"

#include <math.h>

#define STRAIGHT_TURN_CONTROL_PERIOD_S       (0.005f)
#define STRAIGHT_TURN_DISTANCE_M             (1.500f)
#define STRAIGHT_TURN_SPEED_MPS              (0.20f)
#define STRAIGHT_TURN_ACCELERATION_MPS2      (0.30f)
#define STRAIGHT_TURN_LINE_HEADING_GAIN_DEG  (3.0f)
#define STRAIGHT_TURN_MAX_HEADING_OFFSET_DEG (6.0f)
#define STRAIGHT_TURN_HEADING_KP              (0.025f)
#define STRAIGHT_TURN_GYRO_KD                 (0.0020f)
#define STRAIGHT_TURN_MAX_OMEGA_RAD_S         (0.22f)
#define STRAIGHT_TURN_LINE_FILTER_GAIN        (0.08f)
#define STRAIGHT_TURN_MAX_DISTANCE_STEP_M     (0.005f)
#define STRAIGHT_TURN_LINE_LOST_TICKS         (200U)
#define STRAIGHT_TURN_IMU_STALE_TICKS         (200U)
#define STRAIGHT_TURN_TIMEOUT_TICKS           (4000U)
#define STRAIGHT_TURN_ARC_TARGET_DEG          (179.0f)

volatile StraightTurnState_t StraightTurnState =
    STRAIGHT_TURN_IDLE;
volatile uint8_t StraightTurnFault = STRAIGHT_TURN_FAULT_NONE;
volatile float StraightTurnDistanceM;
volatile float StraightTurnYawDeg;
volatile float StraightTurnLineError;
volatile float StraightTurnHeadingErrorDeg;
volatile float StraightTurnCommandSpeed;
volatile uint32_t StraightTurnElapsedMs;

static float straight_turn_start_yaw;
static float straight_turn_filtered_line_error;
static float straight_turn_gyro_z_dps;
static uint32_t straight_turn_last_imu_sample;
static uint16_t straight_turn_imu_stale_ticks;
static uint16_t straight_turn_line_lost_ticks;
static uint16_t straight_turn_elapsed_ticks;

static float straight_turn_limit(
    float value,
    float minimum,
    float maximum)
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

static float straight_turn_wrap_180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void straight_turn_fault(uint8_t fault)
{
    StraightTurnFault = fault;
    StraightTurnState = STRAIGHT_TURN_FAULT;
    StraightTurnCommandSpeed = 0.0f;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    Flag_Stop = 1U;
}

void StraightTurnTest_Reset(void)
{
    StraightTurnState = STRAIGHT_TURN_IDLE;
    StraightTurnFault = STRAIGHT_TURN_FAULT_NONE;
    StraightTurnDistanceM = 0.0f;
    StraightTurnYawDeg = 0.0f;
    StraightTurnLineError = 0.0f;
    StraightTurnHeadingErrorDeg = 0.0f;
    StraightTurnCommandSpeed = 0.0f;
    StraightTurnElapsedMs = 0U;
    straight_turn_start_yaw = 0.0f;
    straight_turn_filtered_line_error = 0.0f;
    straight_turn_gyro_z_dps = 0.0f;
    straight_turn_last_imu_sample = 0U;
    straight_turn_imu_stale_ticks = 0U;
    straight_turn_line_lost_ticks = 0U;
    straight_turn_elapsed_ticks = 0U;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    TurnCalibration_Reset();
}

void StraightTurnTest_Start(void)
{
    imu_sample_t sample;

    StraightTurnTest_Reset();
    imu_get_snapshot(&sample);
    if (sample.status != IMU_STATUS_READY || sample.valid == 0U)
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_IMU);
        return;
    }

    straight_turn_start_yaw = sample.yaw_continuous_deg;
    straight_turn_last_imu_sample = sample.sample_count;
    StraightTurnState = STRAIGHT_TURN_STRAIGHT;
    Flag_Stop = 0U;
}

void StraightTurnTest_Stop(void)
{
    StraightTurnCommandSpeed = 0.0f;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    StraightTurnState = STRAIGHT_TURN_IDLE;
    TurnCalibration_Stop();
    Flag_Stop = 1U;
}

void StraightTurnTest_Run(void)
{
    imu_sample_t sample;
    float line_error;
    float target_heading_deg;
    float command_omega;
    float distance_step;
    float speed_step;
    uint8_t line_valid;

    if (StraightTurnState == STRAIGHT_TURN_ARC)
    {
        TurnCalibration_Run();
        StraightTurnYawDeg = TurnCalibrationYawDeg;
        StraightTurnCommandSpeed =
            TurnCalibrationCommandSpeed;
        StraightTurnElapsedMs =
            (uint32_t)straight_turn_elapsed_ticks * 5U +
            TurnCalibrationElapsedMs;

        if (TurnCalibrationState == TURN_CALIBRATION_DONE)
        {
            StraightTurnState = STRAIGHT_TURN_DONE;
        }
        else if (TurnCalibrationState ==
                 TURN_CALIBRATION_FAULT)
        {
            StraightTurnFault = STRAIGHT_TURN_FAULT_ARC;
            StraightTurnState = STRAIGHT_TURN_FAULT;
        }
        return;
    }

    if (StraightTurnState != STRAIGHT_TURN_STRAIGHT)
    {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        return;
    }

    imu_get_snapshot(&sample);
    if (sample.status == IMU_STATUS_READY &&
        sample.valid != 0U &&
        sample.sample_count != straight_turn_last_imu_sample)
    {
        straight_turn_last_imu_sample = sample.sample_count;
        straight_turn_imu_stale_ticks = 0U;
        StraightTurnYawDeg =
            sample.yaw_continuous_deg - straight_turn_start_yaw;
        straight_turn_gyro_z_dps = sample.gyro_z_dps;
    }
    else if (straight_turn_imu_stale_ticks <
             STRAIGHT_TURN_IMU_STALE_TICKS)
    {
        straight_turn_imu_stale_ticks++;
    }
    else
    {
        straight_turn_fault(
            STRAIGHT_TURN_FAULT_IMU_STALE);
        return;
    }

    line_valid = IR_ReadLineErrorFour(&line_error);
    if (line_valid != 0U)
    {
        straight_turn_line_lost_ticks = 0U;
        straight_turn_filtered_line_error +=
            STRAIGHT_TURN_LINE_FILTER_GAIN *
            (line_error - straight_turn_filtered_line_error);
    }
    else if (straight_turn_line_lost_ticks <
             STRAIGHT_TURN_LINE_LOST_TICKS)
    {
        straight_turn_line_lost_ticks++;
        straight_turn_filtered_line_error *= 0.98f;
    }
    else
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_LINE_LOST);
        return;
    }
    StraightTurnLineError = straight_turn_filtered_line_error;

    distance_step =
        0.5f * (MotorA.Current_Encoder + MotorB.Current_Encoder) *
        STRAIGHT_TURN_CONTROL_PERIOD_S;
    StraightTurnDistanceM +=
        straight_turn_limit(
            distance_step,
            0.0f,
            STRAIGHT_TURN_MAX_DISTANCE_STEP_M);

    straight_turn_elapsed_ticks++;
    StraightTurnElapsedMs =
        (uint32_t)straight_turn_elapsed_ticks * 5U;
    if (straight_turn_elapsed_ticks >=
        STRAIGHT_TURN_TIMEOUT_TICKS)
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_TIMEOUT);
        return;
    }

    if (StraightTurnDistanceM >= STRAIGHT_TURN_DISTANCE_M)
    {
        StraightTurnState = STRAIGHT_TURN_ARC;
        TurnCalibration_StartMovingFromYaw(
            STRAIGHT_TURN_ARC_TARGET_DEG,
            StraightTurnCommandSpeed,
            straight_turn_start_yaw + StraightTurnYawDeg,
            straight_turn_last_imu_sample);
        return;
    }

    target_heading_deg =
        straight_turn_start_yaw +
        straight_turn_limit(
            STRAIGHT_TURN_LINE_HEADING_GAIN_DEG *
                StraightTurnLineError,
            -STRAIGHT_TURN_MAX_HEADING_OFFSET_DEG,
            STRAIGHT_TURN_MAX_HEADING_OFFSET_DEG);
    StraightTurnHeadingErrorDeg =
        straight_turn_wrap_180(
            target_heading_deg -
            (straight_turn_start_yaw + StraightTurnYawDeg));
    command_omega =
        STRAIGHT_TURN_HEADING_KP *
            StraightTurnHeadingErrorDeg -
        STRAIGHT_TURN_GYRO_KD *
            straight_turn_gyro_z_dps;
    command_omega =
        straight_turn_limit(
            command_omega,
            -STRAIGHT_TURN_MAX_OMEGA_RAD_S,
            STRAIGHT_TURN_MAX_OMEGA_RAD_S);

    speed_step =
        STRAIGHT_TURN_ACCELERATION_MPS2 *
        STRAIGHT_TURN_CONTROL_PERIOD_S;
    if (StraightTurnCommandSpeed <
        STRAIGHT_TURN_SPEED_MPS - speed_step)
    {
        StraightTurnCommandSpeed += speed_step;
    }
    else
    {
        StraightTurnCommandSpeed =
            STRAIGHT_TURN_SPEED_MPS;
    }
    Get_Target_Encoder(
        StraightTurnCommandSpeed,
        command_omega);
}

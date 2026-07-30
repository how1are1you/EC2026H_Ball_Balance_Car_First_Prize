#include "straight_turn_test.h"

#include "control.h"
#include "IR_Module.h"
#include "imu/imu.h"
#include "turn_calibration.h"

#include <math.h>

#define STRAIGHT_TURN_CONTROL_PERIOD_S (0.005f)
#define STRAIGHT_TURN_DISTANCE_M (1.500f)
#define STRAIGHT_TURN_ARC_LEAD_M (0.030f)
#define STRAIGHT_TURN_ACCELERATION_MPS2 (0.30f)
#define STRAIGHT_TURN_HEADING_KP (0.025f)
#define STRAIGHT_TURN_GYRO_KD (0.0020f)
#define STRAIGHT_TURN_LINE_OMEGA_KP (0.10f)
#define STRAIGHT_TURN_MAX_OMEGA_RAD_S (0.45f)
#define STRAIGHT_TURN_LINE_FILTER_GAIN (0.08f)
#define STRAIGHT_TURN_MAX_DISTANCE_STEP_M (0.005f)
#define STRAIGHT_TURN_LINE_LOST_TICKS (200U)
#define STRAIGHT_TURN_IMU_STALE_TICKS (200U)
#define STRAIGHT_TURN_TIMEOUT_TICKS (8000U)
#define STRAIGHT_TURN_ARC_TARGET_DEG (177.0f)
#define STRAIGHT_TURN_ARC_BLEND_DEG (4.0f)
#define STRAIGHT_TURN_ARC_STOP_MARGIN_DEG (0.5f)
#define STRAIGHT_TURN_SECOND_STRAIGHT_HEADING_DEG (-177.0f)

volatile StraightTurnState_t StraightTurnState =
    STRAIGHT_TURN_IDLE;
volatile uint8_t StraightTurnFault = STRAIGHT_TURN_FAULT_NONE;
volatile float StraightTurnDistanceM;
volatile float StraightTurnYawDeg;
volatile float StraightTurnLineError;
volatile float StraightTurnHeadingErrorDeg;
volatile float StraightTurnCommandSpeed;
volatile uint32_t StraightTurnElapsedMs;

static float straight_turn_initial_yaw;
static float straight_turn_straight_heading_yaw;
static float straight_turn_arc_start_yaw;
static float straight_turn_last_valid_yaw;
static float straight_turn_filtered_line_error;
static float straight_turn_gyro_z_dps;
static uint32_t straight_turn_last_imu_sample;
static uint16_t straight_turn_imu_stale_ticks;
static uint16_t straight_turn_line_lost_ticks;
static uint16_t straight_turn_elapsed_ticks;
static float straight_turn_target_speed_mps =
    STRAIGHT_TURN_FAST_SPEED_MPS;

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

static uint8_t straight_turn_update_imu(void)
{
    imu_sample_t sample;

    imu_get_snapshot(&sample);
    if (sample.status == IMU_STATUS_READY &&
        sample.valid != 0U &&
        sample.sample_count != straight_turn_last_imu_sample)
    {
        straight_turn_last_imu_sample = sample.sample_count;
        straight_turn_imu_stale_ticks = 0U;
        straight_turn_last_valid_yaw =
            sample.yaw_continuous_deg;
        straight_turn_gyro_z_dps = sample.gyro_z_dps;
        StraightTurnYawDeg =
            straight_turn_last_valid_yaw -
            straight_turn_initial_yaw;
        return 1U;
    }

    if (straight_turn_imu_stale_ticks <
        STRAIGHT_TURN_IMU_STALE_TICKS)
    {
        straight_turn_imu_stale_ticks++;
        return 1U;
    }

    straight_turn_fault(STRAIGHT_TURN_FAULT_IMU_STALE);
    return 0U;
}

static uint8_t straight_turn_update_line(void)
{
    float line_error;

    if (IR_ReadLineErrorFour(&line_error) != 0U)
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
        return 0U;
    }

    StraightTurnLineError =
        straight_turn_filtered_line_error;
    return 1U;
}

static float straight_turn_heading_omega(
    float heading_yaw,
    float line_error)
{
    float command_omega;

    StraightTurnHeadingErrorDeg =
        straight_turn_wrap_180(
            heading_yaw - straight_turn_last_valid_yaw);
    command_omega =
        STRAIGHT_TURN_HEADING_KP *
            StraightTurnHeadingErrorDeg -
        STRAIGHT_TURN_GYRO_KD * straight_turn_gyro_z_dps +
        STRAIGHT_TURN_LINE_OMEGA_KP * line_error;
    return straight_turn_limit(
        command_omega,
        -STRAIGHT_TURN_MAX_OMEGA_RAD_S,
        STRAIGHT_TURN_MAX_OMEGA_RAD_S);
}

static void straight_turn_command_straight(void)
{
    float command_omega =
        straight_turn_heading_omega(
            straight_turn_straight_heading_yaw,
            straight_turn_filtered_line_error);
    float speed_step =
        STRAIGHT_TURN_ACCELERATION_MPS2 *
        STRAIGHT_TURN_CONTROL_PERIOD_S;

    if (StraightTurnCommandSpeed <
        straight_turn_target_speed_mps - speed_step)
    {
        StraightTurnCommandSpeed += speed_step;
    }
    else
    {
        StraightTurnCommandSpeed = straight_turn_target_speed_mps;
    }
    Get_Target_Encoder(
        StraightTurnCommandSpeed,
        command_omega);
}

static void straight_turn_start_arc(uint8_t first_arc)
{
    straight_turn_arc_start_yaw =
        straight_turn_last_valid_yaw;

    if (first_arc != 0U)
    {
        StraightTurnState = STRAIGHT_TURN_ARC_1;
        TurnCalibration_StartMovingFromYawContinuous(
            STRAIGHT_TURN_ARC_TARGET_DEG,
            StraightTurnCommandSpeed,
            straight_turn_target_speed_mps,
            straight_turn_arc_start_yaw,
            straight_turn_last_imu_sample);
    }
    else
    {
        StraightTurnState = STRAIGHT_TURN_ARC_2;
        TurnCalibration_StartMovingFromYaw(
            STRAIGHT_TURN_ARC_TARGET_DEG,
            StraightTurnCommandSpeed,
            straight_turn_target_speed_mps,
            straight_turn_arc_start_yaw,
            straight_turn_last_imu_sample);
    }
}

static void straight_turn_run_straight(void)
{
    float distance_step;
    float turn_start_distance;
    uint8_t first_straight =
        (StraightTurnState == STRAIGHT_TURN_STRAIGHT_1) ? 1U : 0U;

    if (straight_turn_update_imu() == 0U ||
        straight_turn_update_line() == 0U)
    {
        return;
    }

    distance_step =
        0.5f * (MotorA.Current_Encoder + MotorB.Current_Encoder) *
        STRAIGHT_TURN_CONTROL_PERIOD_S;
    StraightTurnDistanceM +=
        straight_turn_limit(
            distance_step,
            0.0f,
            STRAIGHT_TURN_MAX_DISTANCE_STEP_M);

    turn_start_distance =
        STRAIGHT_TURN_DISTANCE_M -
        STRAIGHT_TURN_ARC_LEAD_M;
    if (StraightTurnDistanceM >= turn_start_distance)
    {
        straight_turn_start_arc(first_straight);
        return;
    }

    straight_turn_command_straight();
}

static void straight_turn_blend_arc_exit(float yaw_magnitude)
{
    float blend_start =
        STRAIGHT_TURN_ARC_TARGET_DEG -
        STRAIGHT_TURN_ARC_BLEND_DEG;
    float blend_end =
        STRAIGHT_TURN_ARC_TARGET_DEG -
        STRAIGHT_TURN_ARC_STOP_MARGIN_DEG;
    float blend =
        straight_turn_limit(
            (yaw_magnitude - blend_start) /
                (blend_end - blend_start),
            0.0f,
            1.0f);
    float arc_omega =
        -TurnCalibrationCommandSpeed /
        TurnCalibrationRadiusM;
    float straight_omega =
        straight_turn_heading_omega(
            straight_turn_initial_yaw +
                STRAIGHT_TURN_SECOND_STRAIGHT_HEADING_DEG,
            0.0f);
    float command_omega =
        (1.0f - blend) * arc_omega +
        blend * straight_omega;

    Get_Target_Encoder(
        TurnCalibrationCommandSpeed,
        command_omega);
}

static void straight_turn_run_arc(void)
{
    uint8_t first_arc =
        (StraightTurnState == STRAIGHT_TURN_ARC_1) ? 1U : 0U;
    float yaw_magnitude;

    TurnCalibration_Run();
    StraightTurnCommandSpeed =
        TurnCalibrationCommandSpeed;
    if (straight_turn_update_imu() == 0U)
    {
        return;
    }

    yaw_magnitude = fabsf(TurnCalibrationYawDeg);
    if (first_arc != 0U &&
        yaw_magnitude >=
            STRAIGHT_TURN_ARC_TARGET_DEG -
                STRAIGHT_TURN_ARC_BLEND_DEG)
    {
        straight_turn_blend_arc_exit(yaw_magnitude);
    }

    if (TurnCalibrationState == TURN_CALIBRATION_DONE)
    {
        if (first_arc != 0U)
        {
            StraightTurnState = STRAIGHT_TURN_STRAIGHT_2;
            StraightTurnDistanceM = 0.0f;
            straight_turn_straight_heading_yaw =
                straight_turn_initial_yaw +
                STRAIGHT_TURN_SECOND_STRAIGHT_HEADING_DEG;
            straight_turn_filtered_line_error = 0.0f;
            straight_turn_line_lost_ticks = 0U;
            straight_turn_command_straight();
        }
        else
        {
            StraightTurnState = STRAIGHT_TURN_DONE;
        }
    }
    else if (TurnCalibrationState == TURN_CALIBRATION_FAULT)
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_ARC);
    }
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
    straight_turn_initial_yaw = 0.0f;
    straight_turn_straight_heading_yaw = 0.0f;
    straight_turn_arc_start_yaw = 0.0f;
    straight_turn_last_valid_yaw = 0.0f;
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

void StraightTurnTest_Start(float target_speed_mps)
{
    imu_sample_t sample;

    StraightTurnTest_Reset();
    straight_turn_target_speed_mps =
        straight_turn_limit(target_speed_mps, 0.05f, 0.50f);
    imu_get_snapshot(&sample);
    if (sample.status != IMU_STATUS_READY || sample.valid == 0U)
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_IMU);
        return;
    }

    straight_turn_initial_yaw = sample.yaw_continuous_deg;
    straight_turn_straight_heading_yaw =
        straight_turn_initial_yaw;
    straight_turn_last_valid_yaw = sample.yaw_continuous_deg;
    straight_turn_gyro_z_dps = sample.gyro_z_dps;
    straight_turn_last_imu_sample = sample.sample_count;
    StraightTurnState = STRAIGHT_TURN_STRAIGHT_1;
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
    if (StraightTurnState == STRAIGHT_TURN_IDLE ||
        StraightTurnState == STRAIGHT_TURN_DONE ||
        StraightTurnState == STRAIGHT_TURN_FAULT)
    {
        return;
    }

    straight_turn_elapsed_ticks++;
    StraightTurnElapsedMs =
        (uint32_t)straight_turn_elapsed_ticks * 5U;
    if (straight_turn_elapsed_ticks >=
        STRAIGHT_TURN_TIMEOUT_TICKS)
    {
        straight_turn_fault(STRAIGHT_TURN_FAULT_TIMEOUT);
        return;
    }

    if (StraightTurnState == STRAIGHT_TURN_STRAIGHT_1 ||
        StraightTurnState == STRAIGHT_TURN_STRAIGHT_2)
    {
        straight_turn_run_straight();
    }
    else
    {
        straight_turn_run_arc();
    }
}

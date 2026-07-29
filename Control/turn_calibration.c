#include "turn_calibration.h"

#include "control.h"
#include "imu/imu.h"

#include <math.h>

#define TURN_CALIBRATION_CONTROL_PERIOD_S  (0.005f)
#define TURN_CALIBRATION_SPEED_MPS         (0.20f)
#define TURN_CALIBRATION_ACCELERATION_MPS2 (0.30f)
#define TURN_CALIBRATION_DECELERATION_MPS2 (0.40f)
#define TURN_CALIBRATION_TARGET_YAW_DEG    (180.0f)
#define TURN_CALIBRATION_STOP_MARGIN_DEG   (0.5f)
#define TURN_CALIBRATION_CLOCKWISE_SIGN    (-1.0f)
#define TURN_CALIBRATION_RADIUS_MIN_M      (0.30f)
#define TURN_CALIBRATION_RADIUS_MAX_M      (0.70f)
#define TURN_CALIBRATION_RADIUS_STEP_M     (0.02f)
#define TURN_CALIBRATION_TIMEOUT_TICKS     (2400U)
#define TURN_CALIBRATION_IMU_STALE_TICKS   (200U)
#define TURN_CALIBRATION_MAX_DISTANCE_STEP (0.005f)
#define TURN_CALIBRATION_PI                (3.1415926f)

volatile TurnCalibrationState_t TurnCalibrationState =
    TURN_CALIBRATION_IDLE;
volatile uint8_t TurnCalibrationFault = TURN_CALIBRATION_FAULT_NONE;
volatile float TurnCalibrationRadiusM =
    TURN_CALIBRATION_RADIUS_DEFAULT_M;
volatile float TurnCalibrationTargetYawDeg =
    TURN_CALIBRATION_TARGET_YAW_DEG;
volatile float TurnCalibrationYawDeg;
volatile float TurnCalibrationDistanceM;
volatile float TurnCalibrationCommandSpeed;
volatile uint32_t TurnCalibrationElapsedMs;

static float turn_calibration_start_yaw;
static uint32_t turn_calibration_last_sample_count;
static uint16_t turn_calibration_elapsed_ticks;
static uint16_t turn_calibration_imu_stale_ticks;
static uint8_t turn_calibration_stop_on_complete;

static float turn_calibration_limit(
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

static void turn_calibration_set_targets(float speed)
{
    float omega =
        TURN_CALIBRATION_CLOCKWISE_SIGN * speed /
        TurnCalibrationRadiusM;

    MotorA.Target_Encoder =
        speed - 0.5f * DRIVE_WHEEL_SPACING * omega;
    MotorB.Target_Encoder =
        speed + 0.5f * DRIVE_WHEEL_SPACING * omega;
}

void TurnCalibration_Reset(void)
{
    TurnCalibrationState = TURN_CALIBRATION_IDLE;
    TurnCalibrationFault = TURN_CALIBRATION_FAULT_NONE;
    TurnCalibrationTargetYawDeg =
        TURN_CALIBRATION_TARGET_YAW_DEG;
    TurnCalibrationYawDeg = 0.0f;
    TurnCalibrationDistanceM = 0.0f;
    TurnCalibrationCommandSpeed = 0.0f;
    TurnCalibrationElapsedMs = 0U;
    turn_calibration_start_yaw = 0.0f;
    turn_calibration_last_sample_count = 0U;
    turn_calibration_elapsed_ticks = 0U;
    turn_calibration_imu_stale_ticks = 0U;
    turn_calibration_stop_on_complete = 1U;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
}

void TurnCalibration_Start(void)
{
    TurnCalibration_StartAngle(
        TURN_CALIBRATION_TARGET_YAW_DEG);
}

void TurnCalibration_StartAngle(float target_yaw_deg)
{
    TurnCalibration_StartMoving(target_yaw_deg, 0.0f);
}

void TurnCalibration_StartMoving(
    float target_yaw_deg,
    float initial_speed_mps)
{
    imu_sample_t sample;

    TurnCalibration_Reset();
    imu_get_snapshot(&sample);
    if (sample.status != IMU_STATUS_READY || sample.valid == 0U)
    {
        TurnCalibrationState = TURN_CALIBRATION_FAULT;
        TurnCalibrationFault = TURN_CALIBRATION_FAULT_IMU;
        Flag_Stop = 1U;
        return;
    }

    turn_calibration_start_yaw = sample.yaw_continuous_deg;
    turn_calibration_last_sample_count = sample.sample_count;
    TurnCalibrationTargetYawDeg =
        turn_calibration_limit(target_yaw_deg, 1.0f, 360.0f);
    TurnCalibrationCommandSpeed =
        turn_calibration_limit(
            initial_speed_mps,
            0.0f,
            TURN_CALIBRATION_SPEED_MPS);
    TurnCalibrationState = TURN_CALIBRATION_RUNNING;
    Flag_Stop = 0U;
}

void TurnCalibration_StartMovingFromYaw(
    float target_yaw_deg,
    float initial_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count)
{
    TurnCalibration_Reset();
    turn_calibration_start_yaw = start_yaw_deg;
    turn_calibration_last_sample_count = last_sample_count;
    TurnCalibrationTargetYawDeg =
        turn_calibration_limit(target_yaw_deg, 1.0f, 360.0f);
    TurnCalibrationCommandSpeed =
        turn_calibration_limit(
            initial_speed_mps,
            0.0f,
            TURN_CALIBRATION_SPEED_MPS);
    TurnCalibrationState = TURN_CALIBRATION_RUNNING;
    Flag_Stop = 0U;
}

void TurnCalibration_StartMovingFromYawContinuous(
    float target_yaw_deg,
    float initial_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count)
{
    TurnCalibration_StartMovingFromYaw(
        target_yaw_deg,
        initial_speed_mps,
        start_yaw_deg,
        last_sample_count);
    turn_calibration_stop_on_complete = 0U;
}

void TurnCalibration_Stop(void)
{
    TurnCalibrationCommandSpeed = 0.0f;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
    TurnCalibrationState = TURN_CALIBRATION_IDLE;
    Flag_Stop = 1U;
}

void TurnCalibration_IncreaseRadius(void)
{
    float radius = TurnCalibrationRadiusM +
                   TURN_CALIBRATION_RADIUS_STEP_M;

    if (radius > TURN_CALIBRATION_RADIUS_MAX_M + 0.001f)
    {
        radius = TURN_CALIBRATION_RADIUS_MIN_M;
    }
    TurnCalibrationRadiusM = radius;
    TurnCalibration_Reset();
}

void TurnCalibration_Run(void)
{
    imu_sample_t sample;
    float distance_step;
    float yaw_magnitude;
    float remaining_angle_rad;
    float remaining_arc_m;
    float desired_speed;
    float braking_speed;
    float speed_step;

    if (TurnCalibrationState != TURN_CALIBRATION_RUNNING &&
        TurnCalibrationState != TURN_CALIBRATION_BRAKING)
    {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        return;
    }

    imu_get_snapshot(&sample);
    if (sample.status == IMU_STATUS_READY &&
        sample.valid != 0U &&
        sample.sample_count !=
            turn_calibration_last_sample_count)
    {
        turn_calibration_last_sample_count = sample.sample_count;
        turn_calibration_imu_stale_ticks = 0U;
        TurnCalibrationYawDeg =
            sample.yaw_continuous_deg - turn_calibration_start_yaw;
    }
    else if (turn_calibration_imu_stale_ticks <
             TURN_CALIBRATION_IMU_STALE_TICKS)
    {
        turn_calibration_imu_stale_ticks++;
    }
    else
    {
        TurnCalibrationCommandSpeed = 0.0f;
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        TurnCalibrationState = TURN_CALIBRATION_FAULT;
        TurnCalibrationFault = TURN_CALIBRATION_FAULT_IMU;
        Flag_Stop = 1U;
        return;
    }
    yaw_magnitude = fabsf(TurnCalibrationYawDeg);

    distance_step =
        0.5f * (MotorA.Current_Encoder + MotorB.Current_Encoder) *
        TURN_CALIBRATION_CONTROL_PERIOD_S;
    TurnCalibrationDistanceM +=
        turn_calibration_limit(
            distance_step,
            0.0f,
            TURN_CALIBRATION_MAX_DISTANCE_STEP);

    turn_calibration_elapsed_ticks++;
    TurnCalibrationElapsedMs =
        (uint32_t)turn_calibration_elapsed_ticks * 5U;

    if (yaw_magnitude >=
        TurnCalibrationTargetYawDeg -
            TURN_CALIBRATION_STOP_MARGIN_DEG)
    {
        TurnCalibrationState = TURN_CALIBRATION_DONE;
        if (turn_calibration_stop_on_complete != 0U)
        {
            TurnCalibrationCommandSpeed = 0.0f;
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
            Flag_Stop = 1U;
        }
        return;
    }

    if (turn_calibration_elapsed_ticks >=
        TURN_CALIBRATION_TIMEOUT_TICKS)
    {
        TurnCalibrationCommandSpeed = 0.0f;
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        TurnCalibrationState = TURN_CALIBRATION_FAULT;
        TurnCalibrationFault = TURN_CALIBRATION_FAULT_TIMEOUT;
        Flag_Stop = 1U;
        return;
    }

    remaining_angle_rad =
        (TurnCalibrationTargetYawDeg - yaw_magnitude) *
        TURN_CALIBRATION_PI / 180.0f;
    remaining_arc_m =
        TurnCalibrationRadiusM * remaining_angle_rad;
    desired_speed = TURN_CALIBRATION_SPEED_MPS;
    if (turn_calibration_stop_on_complete != 0U)
    {
        braking_speed = sqrtf(
            2.0f * TURN_CALIBRATION_DECELERATION_MPS2 *
            remaining_arc_m);
        if (braking_speed < desired_speed)
        {
            desired_speed = braking_speed;
            TurnCalibrationState = TURN_CALIBRATION_BRAKING;
        }
    }

    speed_step =
        ((desired_speed >= TurnCalibrationCommandSpeed) ?
             TURN_CALIBRATION_ACCELERATION_MPS2 :
             TURN_CALIBRATION_DECELERATION_MPS2) *
        TURN_CALIBRATION_CONTROL_PERIOD_S;
    if (desired_speed >
        TurnCalibrationCommandSpeed + speed_step)
    {
        TurnCalibrationCommandSpeed += speed_step;
    }
    else if (desired_speed <
             TurnCalibrationCommandSpeed - speed_step)
    {
        TurnCalibrationCommandSpeed -= speed_step;
    }
    else
    {
        TurnCalibrationCommandSpeed = desired_speed;
    }

    turn_calibration_set_targets(TurnCalibrationCommandSpeed);
}

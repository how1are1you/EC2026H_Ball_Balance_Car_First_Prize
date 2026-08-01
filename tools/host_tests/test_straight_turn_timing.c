#include <assert.h>
#include <stdint.h>

#define __CONTROL_H

typedef struct
{
    float Current_Encoder;
    float Motor_Pwm;
    float Target_Encoder;
    float Velocity;
} Motor_parameter;

Motor_parameter MotorA;
Motor_parameter MotorB;
uint8_t Flag_Stop = 1U;

void Get_Target_Encoder(float speed_mps, float omega_rad_s)
{
    MotorA.Target_Encoder = speed_mps - omega_rad_s;
    MotorB.Target_Encoder = speed_mps + omega_rad_s;
}

#define IR_MODULE_H

uint8_t IR_ReadLineErrorFour(float *error)
{
    *error = 0.0f;
    return 1U;
}

#define IMU_H

typedef enum
{
    IMU_STATUS_NOT_INITIALIZED = 0,
    IMU_STATUS_INITIALIZING,
    IMU_STATUS_READY,
    IMU_STATUS_DEVICE_NOT_FOUND,
    IMU_STATUS_DMP_ERROR,
    IMU_STATUS_FIFO_ERROR
} imu_status_t;

typedef struct
{
    imu_status_t status;
    uint8_t valid;
    uint8_t who_am_i;
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_continuous_deg;
    float gyro_z_dps;
    float accel_g[3];
    uint32_t timestamp_ms;
    uint32_t interrupt_count;
    uint32_t sample_count;
    uint32_t fifo_error_count;
} imu_sample_t;

static imu_sample_t fake_imu_sample;

void imu_get_snapshot(imu_sample_t *sample)
{
    *sample = fake_imu_sample;
}

#define STARTUP_MOTION_GATE_H

typedef struct
{
    uint8_t consecutive_motion_ticks;
    uint8_t motion_detected;
} startup_motion_gate_t;

void startup_motion_gate_reset(startup_motion_gate_t *gate)
{
    gate->consecutive_motion_ticks = 0U;
    gate->motion_detected = 0U;
}

uint8_t startup_motion_gate_update(
    startup_motion_gate_t *gate,
    float left_speed_mps,
    float right_speed_mps)
{
    (void)left_speed_mps;
    (void)right_speed_mps;
    gate->motion_detected = 1U;
    return 1U;
}

#define TURN_CALIBRATION_H

typedef enum
{
    TURN_CALIBRATION_IDLE = 0,
    TURN_CALIBRATION_RUNNING,
    TURN_CALIBRATION_BRAKING,
    TURN_CALIBRATION_DONE,
    TURN_CALIBRATION_FAULT
} TurnCalibrationState_t;

volatile TurnCalibrationState_t TurnCalibrationState =
    TURN_CALIBRATION_IDLE;
volatile float TurnCalibrationRadiusM = 0.471f;
volatile float TurnCalibrationYawDeg;
volatile float TurnCalibrationCommandSpeed;

void TurnCalibration_Run(void)
{
}

void TurnCalibration_StartMovingFromYaw(
    float target_yaw_deg,
    float initial_speed_mps,
    float cruise_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count)
{
    (void)target_yaw_deg;
    (void)cruise_speed_mps;
    (void)start_yaw_deg;
    (void)last_sample_count;
    TurnCalibrationCommandSpeed = initial_speed_mps;
    TurnCalibrationState = TURN_CALIBRATION_RUNNING;
}

void TurnCalibration_StartMovingFromYawContinuous(
    float target_yaw_deg,
    float initial_speed_mps,
    float cruise_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count)
{
    TurnCalibration_StartMovingFromYaw(
        target_yaw_deg,
        initial_speed_mps,
        cruise_speed_mps,
        start_yaw_deg,
        last_sample_count);
}

void TurnCalibration_Stop(void)
{
    TurnCalibrationState = TURN_CALIBRATION_IDLE;
}

void TurnCalibration_Reset(void)
{
    TurnCalibrationState = TURN_CALIBRATION_IDLE;
    TurnCalibrationYawDeg = 0.0f;
    TurnCalibrationCommandSpeed = 0.0f;
}

#include "../../Control/straight_turn_test.c"

int main(void)
{
    uint32_t locked_time_ms;
    uint16_t tick_count = 0U;

    fake_imu_sample.status = IMU_STATUS_READY;
    fake_imu_sample.valid = 1U;
    fake_imu_sample.sample_count = 1U;
    MotorA.Current_Encoder = 0.5f;
    MotorB.Current_Encoder = 0.5f;

    StraightTurnTest_StartWithPostLap(
        STRAIGHT_TURN_BALL_SPEED_MPS,
        STRAIGHT_TURN_BALL_ACCELERATION_MPS2,
        STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M);

    while (StraightTurnState == STRAIGHT_TURN_STRAIGHT_1 &&
           tick_count < 1000U)
    {
        fake_imu_sample.sample_count++;
        StraightTurnTest_Run();
        tick_count++;
    }

    assert(tick_count < 1000U);
    assert(StraightTurnState == STRAIGHT_TURN_ARC_1);
    assert(StraightTurnStraight1TimeMs != 0U);
    assert(StraightTurnStraight1TimeMs == StraightTurnElapsedMs);

    locked_time_ms = StraightTurnStraight1TimeMs;
    fake_imu_sample.sample_count++;
    StraightTurnTest_Run();
    assert(StraightTurnElapsedMs > locked_time_ms);
    assert(StraightTurnStraight1TimeMs == locked_time_ms);

    StraightTurnTest_Reset();
    assert(StraightTurnStraight1TimeMs == 0U);

    return 0;
}

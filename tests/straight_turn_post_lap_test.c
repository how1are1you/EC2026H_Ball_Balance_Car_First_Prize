#include <assert.h>
#include <stdint.h>

#define __CONTROL_H

#define DRIVE_WHEEL_SPACING (0.210f)

typedef struct
{
    float Current_Encoder;
    float Motor_Pwm;
    float Target_Encoder;
    float Velocity;
} Motor_parameter;

Motor_parameter MotorA;
Motor_parameter MotorB;
unsigned char Flag_Stop = 1U;

void Get_Target_Encoder(float speed_mps, float omega_rad_s);

#include "../Control/straight_turn_test.c"

volatile TurnCalibrationState_t TurnCalibrationState =
    TURN_CALIBRATION_IDLE;
volatile uint8_t TurnCalibrationFault;
volatile float TurnCalibrationRadiusM = 0.471f;
volatile float TurnCalibrationTargetYawDeg;
volatile float TurnCalibrationYawDeg;
volatile float TurnCalibrationDistanceM;
volatile float TurnCalibrationCommandSpeed;
volatile uint32_t TurnCalibrationElapsedMs;

static imu_sample_t test_imu_sample;

void imu_get_snapshot(imu_sample_t *sample)
{
    test_imu_sample.sample_count++;
    *sample = test_imu_sample;
}

uint8_t IR_ReadLineErrorFour(float *error)
{
    *error = 0.0f;
    return 1U;
}

void Get_Target_Encoder(float speed_mps, float omega_rad_s)
{
    MotorA.Target_Encoder =
        speed_mps - 0.5f * DRIVE_WHEEL_SPACING * omega_rad_s;
    MotorB.Target_Encoder =
        speed_mps + 0.5f * DRIVE_WHEEL_SPACING * omega_rad_s;
}

void TurnCalibration_Run(void)
{
    TurnCalibrationYawDeg = 177.0f;
    TurnCalibrationState = TURN_CALIBRATION_DONE;
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
    Flag_Stop = 0U;
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

void TurnCalibration_Reset(void)
{
    TurnCalibrationState = TURN_CALIBRATION_IDLE;
    TurnCalibrationCommandSpeed = 0.0f;
    TurnCalibrationYawDeg = 0.0f;
}

void TurnCalibration_Stop(void)
{
    TurnCalibration_Reset();
    Flag_Stop = 1U;
}

static void initialize_test(void)
{
    test_imu_sample.status = IMU_STATUS_READY;
    test_imu_sample.valid = 1U;
    test_imu_sample.sample_count = 1U;
    test_imu_sample.yaw_continuous_deg = 0.0f;
    test_imu_sample.gyro_z_dps = 0.0f;
    MotorA.Current_Encoder = 1.0f;
    MotorB.Current_Encoder = 1.0f;
    StraightTurnTest_Reset();
}

static void second_arc_locks_lap_time_and_starts_post_lap(void)
{
    unsigned int tick;

    initialize_test();
    StraightTurnTest_StartWithPostLap(0.25f, 1.0f, 1.0f);

    for (tick = 0U;
         tick < 1000U &&
         StraightTurnState != STRAIGHT_TURN_POST_LAP;
         tick++)
    {
        StraightTurnTest_Run();
    }

    assert(StraightTurnState == STRAIGHT_TURN_POST_LAP);
    assert(StraightTurnLapTimeMs > 0U);
    assert(StraightTurnLapTimeMs == StraightTurnElapsedMs);
    assert(StraightTurnPostLapDistanceM == 0.0f);
    assert(Flag_Stop == 0U);
}

static void post_lap_distance_then_braking_reaches_done(void)
{
    unsigned int tick;
    uint32_t locked_lap_time;

    initialize_test();
    StraightTurnTest_StartWithPostLap(0.25f, 1.0f, 1.0f);
    for (tick = 0U;
         tick < 1000U &&
         StraightTurnState != STRAIGHT_TURN_POST_LAP;
         tick++)
    {
        StraightTurnTest_Run();
    }
    locked_lap_time = StraightTurnLapTimeMs;

    for (tick = 0U;
         tick < 1000U &&
         StraightTurnState != STRAIGHT_TURN_DONE;
         tick++)
    {
        StraightTurnTest_Run();
    }

    assert(StraightTurnState == STRAIGHT_TURN_DONE);
    assert(StraightTurnPostLapDistanceM >= 1.0f);
    assert(StraightTurnLapTimeMs == locked_lap_time);
    assert(StraightTurnElapsedMs > StraightTurnLapTimeMs);
    assert(MotorA.Target_Encoder == 0.0f);
    assert(MotorB.Target_Encoder == 0.0f);
    assert(Flag_Stop == 1U);
}

static void lap_timeout_does_not_abort_post_lap(void)
{
    unsigned int tick;

    initialize_test();
    StraightTurnTest_StartWithPostLap(0.25f, 1.0f, 1.0f);
    for (tick = 0U;
         tick < 1000U &&
         StraightTurnState != STRAIGHT_TURN_POST_LAP;
         tick++)
    {
        StraightTurnTest_Run();
    }

    straight_turn_elapsed_ticks =
        STRAIGHT_TURN_TIMEOUT_TICKS - 1U;
    StraightTurnTest_Run();

    assert(StraightTurnState == STRAIGHT_TURN_POST_LAP);
    assert(StraightTurnFault == STRAIGHT_TURN_FAULT_NONE);
    assert(Flag_Stop == 0U);
}

static void stalled_post_lap_triggers_its_own_watchdog(void)
{
    unsigned int tick;

    initialize_test();
    StraightTurnTest_StartWithPostLap(0.25f, 1.0f, 1.0f);
    for (tick = 0U;
         tick < 1000U &&
         StraightTurnState != STRAIGHT_TURN_POST_LAP;
         tick++)
    {
        StraightTurnTest_Run();
    }

    MotorA.Current_Encoder = 0.0f;
    MotorB.Current_Encoder = 0.0f;
    straight_turn_post_elapsed_ticks =
        STRAIGHT_TURN_POST_TIMEOUT_TICKS - 1U;
    StraightTurnTest_Run();

    assert(StraightTurnState == STRAIGHT_TURN_FAULT);
    assert(StraightTurnFault ==
           STRAIGHT_TURN_FAULT_POST_TIMEOUT);
    assert(Flag_Stop == 1U);
}

static void ball_hold_overtime_before_a_does_not_stop_run(void)
{
    initialize_test();
    StraightTurnTest_StartWithPostLap(0.25f, 1.0f, 1.0f);
    straight_turn_elapsed_ticks =
        STRAIGHT_TURN_TIMEOUT_TICKS - 1U;

    StraightTurnTest_Run();

    assert(StraightTurnState ==
           STRAIGHT_TURN_STRAIGHT_1);
    assert(StraightTurnFault ==
           STRAIGHT_TURN_FAULT_NONE);
    assert(Flag_Stop == 0U);
}

static void both_straights_start_arc_four_cm_early(void)
{
    initialize_test();
    StraightTurnTest_Start(0.25f, 1.0f);
    StraightTurnDistanceM = 1.455f;

    StraightTurnTest_Run();

    assert(StraightTurnState == STRAIGHT_TURN_ARC_1);

    initialize_test();
    StraightTurnTest_Start(0.25f, 1.0f);
    StraightTurnState = STRAIGHT_TURN_STRAIGHT_2;
    StraightTurnDistanceM = 1.455f;

    StraightTurnTest_Run();

    assert(StraightTurnState == STRAIGHT_TURN_ARC_2);
}

int main(void)
{
    second_arc_locks_lap_time_and_starts_post_lap();
    post_lap_distance_then_braking_reaches_done();
    lap_timeout_does_not_abort_post_lap();
    stalled_post_lap_triggers_its_own_watchdog();
    ball_hold_overtime_before_a_does_not_stop_run();
    both_straights_start_arc_four_cm_early();
    return 0;
}

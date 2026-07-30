#ifndef TURN_CALIBRATION_H
#define TURN_CALIBRATION_H

#include <stdint.h>

typedef enum
{
    TURN_CALIBRATION_IDLE = 0,
    TURN_CALIBRATION_RUNNING,
    TURN_CALIBRATION_BRAKING,
    TURN_CALIBRATION_DONE,
    TURN_CALIBRATION_FAULT
} TurnCalibrationState_t;

#define TURN_CALIBRATION_FAULT_NONE (0U)
#define TURN_CALIBRATION_FAULT_IMU (1U)
#define TURN_CALIBRATION_FAULT_TIMEOUT (2U)

/*
 * Change this default after the on-track 180-degree arc test is satisfactory.
 * The OLED tuning value is kept while powered, but is not written to flash.
 */
#define TURN_CALIBRATION_RADIUS_DEFAULT_M (0.475f)

extern volatile TurnCalibrationState_t TurnCalibrationState;
extern volatile uint8_t TurnCalibrationFault;
extern volatile float TurnCalibrationRadiusM;
extern volatile float TurnCalibrationTargetYawDeg;
extern volatile float TurnCalibrationYawDeg;
extern volatile float TurnCalibrationDistanceM;
extern volatile float TurnCalibrationCommandSpeed;
extern volatile uint32_t TurnCalibrationElapsedMs;

void TurnCalibration_Run(void);
void TurnCalibration_Start(void);
void TurnCalibration_StartAngle(float target_yaw_deg);
void TurnCalibration_StartMoving(
    float target_yaw_deg,
    float initial_speed_mps,
    float cruise_speed_mps);
void TurnCalibration_StartMovingFromYaw(
    float target_yaw_deg,
    float initial_speed_mps,
    float cruise_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count);
void TurnCalibration_StartMovingFromYawContinuous(
    float target_yaw_deg,
    float initial_speed_mps,
    float cruise_speed_mps,
    float start_yaw_deg,
    uint32_t last_sample_count);
void TurnCalibration_Stop(void);
void TurnCalibration_Reset(void);
void TurnCalibration_IncreaseRadius(void);

#endif

#ifndef IR_MODULE_H
#define IR_MODULE_H

#include <stdint.h>

extern uint8_t LF04_DH1_State;
extern uint8_t LF04_DH2_State;
extern uint8_t LF04_DH3_State;
extern uint8_t LF04_DH4_State;
extern uint8_t LF04_State;

/*
 * The lap controller reads all four channels. DH1/DH4 are the outer sensors
 * with a 3.0 steering weight; the inner DH2/DH3 sensors use 1.8.
 * Set this to 0 only if a black line is reported as a logic-low input.
 */
extern uint8_t LF04_BlackActiveHigh;

typedef enum
{
    LF04_LAP_IDLE = 0,
    LF04_LAP_STRAIGHT_1,
    LF04_LAP_CURVE_1,
    LF04_LAP_STRAIGHT_2,
    LF04_LAP_CURVE_2,
    LF04_LAP_BRAKING,
    LF04_LAP_SETTLING,
    LF04_LAP_DONE,
    LF04_LAP_FAULT
} LF04_LapState_t;

#define LF04_LAP_FAULT_NONE          (0U)
#define LF04_LAP_FAULT_LINE_LOST     (1U)
#define LF04_LAP_FAULT_TIMEOUT       (2U)
#define LF04_LAP_FAULT_PROGRESS      (3U)
#define LF04_LAP_FAULT_IMU_STALE     (4U)

/* Initial on-board tuning parameters. Units are SI unless noted. */
extern float LF04_BaseSpeed;
extern float LF04_LostLineSpeed;
extern float LF04_SteeringSign;
extern float LF04_Acceleration;
extern float LF04_Deceleration;
extern float LF04_SteeringKp;
extern float LF04_SteeringKi;
extern float LF04_SteeringKd;
extern float LF04_MaxFeedbackOmega;
extern float LF04_CurveFeedforwardGain;
extern float LF04_SensorToDriveAxle;
extern float LF04_LapTargetDistance;
extern float LF04_FinishYawMinimum;

extern volatile LF04_LapState_t LF04_LapState;
extern volatile uint8_t LF04_LapFault;
extern volatile uint8_t LF04_ImuUsed;
extern volatile float LF04_LapDistanceM;
extern volatile float LF04_LapYawDeg;
extern volatile float LF04_SteeringError;
extern volatile float LF04_CommandSpeed;
extern volatile uint32_t LF04_LapElapsedMs;

void IR_Differential_OneLap(void);
void IR_Differential_OneLap_Reset(void);
uint8_t IR_ReadLineErrorFour(float *error);

#endif

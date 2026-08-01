#ifndef STRAIGHT_TURN_TEST_H
#define STRAIGHT_TURN_TEST_H

#include <stdint.h>

typedef enum
{
    STRAIGHT_TURN_IDLE = 0,
    STRAIGHT_TURN_STRAIGHT_1,
    STRAIGHT_TURN_ARC_1,
    STRAIGHT_TURN_STRAIGHT_2,
    STRAIGHT_TURN_ARC_2,
    STRAIGHT_TURN_POST_LAP,
    STRAIGHT_TURN_BRAKING,
    STRAIGHT_TURN_DONE,
    STRAIGHT_TURN_FAULT
} StraightTurnState_t;

#define STRAIGHT_TURN_FAULT_NONE (0U)
#define STRAIGHT_TURN_FAULT_IMU (1U)
#define STRAIGHT_TURN_FAULT_LINE_LOST (2U)
#define STRAIGHT_TURN_FAULT_ARC (3U)
#define STRAIGHT_TURN_FAULT_TIMEOUT (4U)
#define STRAIGHT_TURN_FAULT_IMU_STALE (5U)
#define STRAIGHT_TURN_FAULT_POST_TIMEOUT (6U)

#define STRAIGHT_TURN_FAST_SPEED_MPS (0.35f)
#define STRAIGHT_TURN_BALL_SPEED_MPS (0.25f)
#define STRAIGHT_TURN_FAST_ACCELERATION_MPS2 (0.30f)
#define STRAIGHT_TURN_BALL_ACCELERATION_MPS2 (0.09f)
#define STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M (1.00f)

extern volatile StraightTurnState_t StraightTurnState;
extern volatile uint8_t StraightTurnFault;
extern volatile float StraightTurnDistanceM;
extern volatile float StraightTurnYawDeg;
extern volatile float StraightTurnLineError;
extern volatile float StraightTurnHeadingErrorDeg;
extern volatile float StraightTurnCommandSpeed;
extern volatile float StraightTurnStartupAccelerationMps2;
extern volatile uint32_t StraightTurnElapsedMs;
extern volatile uint32_t StraightTurnStraight1TimeMs;
extern volatile uint32_t StraightTurnLapTimeMs;
extern volatile float StraightTurnPostLapDistanceM;

void StraightTurnTest_Run(void);
void StraightTurnTest_Start(
    float target_speed_mps,
    float acceleration_mps2);
void StraightTurnTest_StartWithPostLap(
    float target_speed_mps,
    float acceleration_mps2,
    float post_lap_distance_m);
void StraightTurnTest_Stop(void);
void StraightTurnTest_Reset(void);

#endif

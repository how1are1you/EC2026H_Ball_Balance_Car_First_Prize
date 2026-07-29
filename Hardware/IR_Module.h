#ifndef IR_MODULE_H
#define IR_MODULE_H

#include <stdint.h>

extern uint8_t LF04_DH1_State;
extern uint8_t LF04_DH2_State;
extern uint8_t LF04_DH3_State;
extern uint8_t LF04_DH4_State;
extern uint8_t LF04_State;

/* Set to 0 if the LF04 module reports a black line with a logic-low output. */
extern uint8_t LF04_BlackActiveHigh;

/* Initial tuning parameters for the Ackermann line-tracking mode. */
extern float LF04_BaseSpeed;
extern float LF04_LostLineSpeed;
extern float LF04_Turn90Angle;
extern float LF04_TurnMaxAngle;
extern float LF04_TurnMinAngle;
extern float LF04_SteeringSign;
extern float LF04_Acceleration;

void IR_Ackermann_LineTrack(void);
void IR_Ackermann_LineTrack_Reset(void);

#endif

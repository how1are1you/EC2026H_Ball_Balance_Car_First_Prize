#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_MIN_PULSE_US (500U)
#define SERVO_NEUTRAL_PULSE_US (1320U)
#define SERVO_MAX_PULSE_US (2500U)

#define SERVO_CONTROL_MIN_PULSE_US (500U)
#define SERVO_CONTROL_MAX_PULSE_US (2200U)
#define SERVO_EFFECTIVE_STEP_US (5U)

extern volatile uint16_t servo_pulse_us;

void servo_init(void);
void servo_set_pulse_us(uint16_t pulse_us);
uint16_t servo_get_pulse_us(void);

#endif

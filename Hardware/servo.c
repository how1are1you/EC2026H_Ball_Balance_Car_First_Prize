#include "servo.h"

#include "ti_msp_dl_config.h"

volatile uint16_t servo_pulse_us = SERVO_NEUTRAL_PULSE_US;

void servo_init(void)
{
    servo_set_pulse_us(SERVO_NEUTRAL_PULSE_US);
}

void servo_set_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US)
    {
        pulse_us = SERVO_MIN_PULSE_US;
    }
    else if (pulse_us > SERVO_MAX_PULSE_US)
    {
        pulse_us = SERVO_MAX_PULSE_US;
    }

    servo_pulse_us = pulse_us;
    DL_Timer_setCaptureCompareValue(
        PWM_1_INST, pulse_us, GPIO_PWM_1_C0_IDX);
}

uint16_t servo_get_pulse_us(void)
{
    return servo_pulse_us;
}

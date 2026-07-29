#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdint.h>

/*
 * Initial commissioning values:
 * - The camera coordinate is assumed to use the beam centre as 0 mm.
 * - Change BALL_BALANCE_SERVO_DIRECTION to -1.0f if the correction
 *   drives the ball farther away from the target.
 * - Keep the first tests within 1300..1700 us, then widen only after
 *   confirming the mechanical limits.
 */
#define BALL_BALANCE_DEFAULT_TARGET_MM       (0.0f)
#define BALL_BALANCE_SERVO_DIRECTION         (1.0f)
#define BALL_BALANCE_KP_US_PER_MM            (1.20f)
#define BALL_BALANCE_KI_US_PER_MM_S          (0.08f)
#define BALL_BALANCE_KD_US_PER_MM_PER_S      (0.35f)
#define BALL_BALANCE_SERVO_MIN_US            (1300U)
#define BALL_BALANCE_SERVO_MAX_US            (1700U)
#define BALL_BALANCE_VISION_TIMEOUT_MS        (200UL)

typedef enum
{
    BALL_BALANCE_DISABLED = 0,
    BALL_BALANCE_WAITING,
    BALL_BALANCE_ACTIVE,
    BALL_BALANCE_STALE
} ball_balance_status_t;

extern volatile float ball_balance_target_mm;
extern volatile uint16_t ball_balance_servo_pulse_us;
extern volatile ball_balance_status_t ball_balance_status;

void ball_balance_init(void);
void ball_balance_set_enabled(uint8_t enabled);
void ball_balance_set_target_mm(float target_mm);
void ball_balance_update(void);

#endif

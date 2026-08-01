#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdint.h>

/*
 * Cascaded controller:
 * position PI -> target velocity -> velocity P with acceleration
 * damping -> servo pulse. All internal position and velocity values
 * use mm and mm/s.
 */
#define BALL_BALANCE_TARGET_MM (0.0f)
#define BALL_BALANCE_SERVO_DIRECTION (1.0f)
#define BALL_BALANCE_POSITION_KP_PER_S (2.0f)
#define BALL_BALANCE_POSITION_KI_PER_S2 (0.05f)
#define BALL_BALANCE_VELOCITY_KP_US_PER_MM_S (4.7f)
#define BALL_BALANCE_VELOCITY_KD_US_PER_MM_S2 (0.12f)
#define BALL_BALANCE_TARGET_VELOCITY_MAX_MM_S (200.0f)
#define BALL_BALANCE_VISION_TIMEOUT_MS (200UL)
#define BALL_BALANCE_REFERENCE_POSITION_LIMIT_MM (110.0f)
#define BALL_BALANCE_REFERENCE_VELOCITY_LIMIT_MM_S (120.0f)
#define BALL_BALANCE_ACCELERATION_FF_DIRECTION (1.0f)
#define BALL_BALANCE_ACCELERATION_FF_US_PER_MPS2 (1150.0f)
#define BALL_BALANCE_ACCELERATION_FF_LIMIT_US (115.0f)
#define BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2 (1.0f)

#define BALL_BALANCE_POSITION_KP_MIN (0.0f)
#define BALL_BALANCE_POSITION_KP_MAX (20.0f)
#define BALL_BALANCE_POSITION_KI_MIN (0.0f)
#define BALL_BALANCE_POSITION_KI_MAX (5.0f)
#define BALL_BALANCE_VELOCITY_KP_MIN (0.0f)
#define BALL_BALANCE_VELOCITY_KP_MAX (20.0f)
#define BALL_BALANCE_VELOCITY_KD_MIN (0.0f)
#define BALL_BALANCE_VELOCITY_KD_MAX (10.0f)
#define BALL_BALANCE_VELOCITY_LIMIT_MIN_MM_S (10.0f)
#define BALL_BALANCE_VELOCITY_LIMIT_MAX_MM_S (500.0f)

typedef enum
{
    BALL_BALANCE_DISABLED = 0,
    BALL_BALANCE_WAITING,
    BALL_BALANCE_ACTIVE,
    BALL_BALANCE_STALE
} ball_balance_status_t;

extern volatile float ball_balance_target_mm;
extern volatile float ball_balance_reference_velocity_mm_s;
extern volatile float ball_balance_position_kp;
extern volatile float ball_balance_position_ki;
extern volatile float ball_balance_velocity_kp;
extern volatile float ball_balance_velocity_kd;
extern volatile float ball_balance_velocity_limit_mm_s;
extern volatile float ball_balance_target_velocity_mm_s;
extern volatile float ball_balance_estimated_position_mm;
extern volatile float ball_balance_measured_velocity_mm_s;
extern volatile float ball_balance_estimated_acceleration_mm_s2;
extern volatile float ball_balance_vehicle_acceleration_mps2;
extern volatile float ball_balance_acceleration_feedforward_us;
extern volatile float ball_balance_position_feedforward_us;
extern volatile float ball_balance_proportional_us;
extern volatile float ball_balance_derivative_us;
extern volatile uint32_t ball_balance_update_count;
extern volatile uint16_t ball_balance_servo_pulse_us;
extern volatile ball_balance_status_t ball_balance_status;

void ball_balance_init(void);
void ball_balance_set_enabled(uint8_t enabled);
void ball_balance_set_vehicle_acceleration(float acceleration_mps2);
void ball_balance_set_reference(
    float position_mm,
    float velocity_mm_s);
uint8_t ball_balance_set_cascade_gains(
    float position_kp,
    float position_ki,
    float velocity_kp,
    float velocity_kd,
    float velocity_limit_mm_s);
void ball_balance_reset_pid_gains(void);
void ball_balance_update(void);

#endif

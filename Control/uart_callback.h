#ifndef UART_CALLBACK_H
#define UART_CALLBACK_H

#include <stdint.h>

/*
 * Vision frame:
 *   0xA5 + position float32 + velocity float32
 * Both float32 values are little-endian. The camera sends centimetres
 * and centimetres/second; this module publishes millimetres and
 * millimetres/second.
 */
extern volatile float vision_ball_position_mm;
extern volatile float vision_ball_velocity_mm_s;
extern volatile uint32_t vision_ball_frame_count;
extern volatile uint32_t vision_ball_last_update_ms;
extern volatile uint32_t vision_uart_error_count;
extern volatile uint8_t vision_ball_position_valid;

typedef enum
{
    CONTROL_UART_DISABLED = 0,
    CONTROL_UART_SERVO_ADJUST,
    CONTROL_UART_PID_TUNING,
    CONTROL_UART_OPEN_LOOP_TUNING
} control_uart_mode_t;

extern volatile uint32_t control_uart_command_count;
extern volatile uint32_t control_uart_error_count;

void vision_uart_reset(void);
void control_uart_reset(void);
void control_uart_set_mode(control_uart_mode_t mode);
void control_uart_service(void);

#endif

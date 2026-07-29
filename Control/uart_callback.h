#ifndef UART_CALLBACK_H
#define UART_CALLBACK_H

#include <stdint.h>

/*
 * Vision frame:
 *   0xA5, D0, D1, D2, D3
 * D0..D3 are an IEEE-754 float32 in little-endian order.
 * The published position is already in millimetres.
 */
extern volatile float vision_ball_position_mm;
extern volatile uint32_t vision_ball_frame_count;
extern volatile uint32_t vision_ball_last_update_ms;
extern volatile uint32_t vision_uart_error_count;
extern volatile uint8_t vision_ball_position_valid;

void vision_uart_reset(void);

#endif

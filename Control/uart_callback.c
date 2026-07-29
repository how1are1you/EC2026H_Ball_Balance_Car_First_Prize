#include "uart_callback.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define VISION_FRAME_HEADER             (0xA5U)
#define VISION_FRAME_PAYLOAD_SIZE       (4U)
#define VISION_FRAME_TIMEOUT_MS         (10UL)
#define FLOAT32_EXPONENT_MASK           (0x7F800000UL)

extern volatile unsigned long tick_ms;

typedef char vision_float_must_be_32_bits[
    (sizeof(float) == VISION_FRAME_PAYLOAD_SIZE) ? 1 : -1];

volatile float vision_ball_position_mm;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint32_t vision_uart_error_count;
volatile uint8_t vision_ball_position_valid;

static volatile uint8_t vision_payload_index;
static uint8_t vision_payload[VISION_FRAME_PAYLOAD_SIZE];
static volatile uint32_t vision_last_byte_ms;

static void vision_uart_reset_parser(void)
{
    vision_payload_index = 0U;
}

static void vision_uart_publish_position(void)
{
    uint32_t raw_value =
        ((uint32_t)vision_payload[0]) |
        ((uint32_t)vision_payload[1] << 8U) |
        ((uint32_t)vision_payload[2] << 16U) |
        ((uint32_t)vision_payload[3] << 24U);
    float position_mm;

    /*
     * The protocol payload is IEEE-754 float32 in little-endian order.
     * Reject NaN and infinity because they cannot represent a position.
     */
    if ((raw_value & FLOAT32_EXPONENT_MASK) == FLOAT32_EXPONENT_MASK)
    {
        vision_uart_error_count++;
        return;
    }

    memcpy(&position_mm, &raw_value, sizeof(position_mm));
    vision_ball_position_mm = position_mm;
    vision_ball_last_update_ms = (uint32_t)tick_ms;
    vision_ball_position_valid = 1U;
    vision_ball_frame_count++;
}

static void vision_uart_process_byte(uint8_t byte)
{
    uint32_t now_ms = (uint32_t)tick_ms;

    if (vision_payload_index != 0U &&
        (uint32_t)(now_ms - vision_last_byte_ms) >
            VISION_FRAME_TIMEOUT_MS)
    {
        vision_uart_reset_parser();
    }
    vision_last_byte_ms = now_ms;

    if (vision_payload_index == 0U)
    {
        if (byte == VISION_FRAME_HEADER)
        {
            vision_payload_index = 1U;
        }
        return;
    }

    vision_payload[vision_payload_index - 1U] = byte;
    vision_payload_index++;
    if (vision_payload_index > VISION_FRAME_PAYLOAD_SIZE)
    {
        vision_uart_reset_parser();
        vision_uart_publish_position();
    }
}

void vision_uart_reset(void)
{
    vision_ball_position_mm = 0.0f;
    vision_ball_frame_count = 0U;
    vision_ball_last_update_ms = 0U;
    vision_uart_error_count = 0U;
    vision_ball_position_valid = 0U;
    vision_last_byte_ms = (uint32_t)tick_ms;
    memset(vision_payload, 0, sizeof(vision_payload));
    vision_uart_reset_parser();
}

void UART_1_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do
    {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(UART_1_INST);

        switch (interrupt_index)
        {
            case DL_UART_MAIN_IIDX_RX:
                while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
                {
                    vision_uart_process_byte(
                        (uint8_t)DL_UART_Main_receiveData(UART_1_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                vision_uart_error_count++;
                vision_uart_reset_parser();
                while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
                {
                    (void)DL_UART_Main_receiveData(UART_1_INST);
                }
                break;

            default:
                break;
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

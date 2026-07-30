#include "uart_callback.h"

#include "ball_balance.h"
#include "ball_static_task.h"
#include "servo.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <string.h>

#define VISION_FRAME_HEADER             (0xA5U)
#define VISION_FLOAT32_SIZE             (4U)
#define VISION_FRAME_PAYLOAD_SIZE       (8U)
#define VISION_POSITION_OFFSET          (0U)
#define VISION_VELOCITY_OFFSET          (4U)
#define VISION_FRAME_TIMEOUT_MS         (10UL)
#define FLOAT32_EXPONENT_MASK           (0x7F800000UL)
#define SERVO_UART_LINE_SIZE            (16U)
#define VISION_POSITION_FILTER_ALPHA    (0.55f)
#define VISION_VELOCITY_FILTER_ALPHA    (0.45f)
#define VISION_FILTER_MIN_DT_MS         (5UL)
#define VISION_FILTER_MAX_DT_MS         (200UL)

extern volatile unsigned long tick_ms;

typedef char vision_float_must_be_32_bits[
    (sizeof(float) == VISION_FLOAT32_SIZE) ? 1 : -1];

volatile float vision_ball_position_mm;
volatile float vision_ball_velocity_mm_s;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint32_t vision_uart_error_count;
volatile uint8_t vision_ball_position_valid;
volatile uint32_t control_uart_command_count;
volatile uint32_t control_uart_error_count;

static volatile uint8_t vision_payload_index;
static uint8_t vision_payload[VISION_FRAME_PAYLOAD_SIZE];
static volatile uint32_t vision_last_byte_ms;
static uint8_t vision_filter_initialized;
static float vision_filtered_position_mm;
static float vision_filtered_velocity_mm_s;
static uint32_t vision_previous_sample_ms;
static volatile control_uart_mode_t control_uart_mode;
static volatile uint8_t servo_uart_rx_length;
static volatile uint8_t servo_uart_command_ready;
static char servo_uart_rx_line[SERVO_UART_LINE_SIZE];
static char servo_uart_command[SERVO_UART_LINE_SIZE];

static void vision_uart_reset_parser(void)
{
    vision_payload_index = 0U;
}

static uint32_t vision_uart_read_u32_le(uint8_t offset)
{
    return
        ((uint32_t)vision_payload[offset]) |
        ((uint32_t)vision_payload[offset + 1U] << 8U) |
        ((uint32_t)vision_payload[offset + 2U] << 16U) |
        ((uint32_t)vision_payload[offset + 3U] << 24U);
}

static void vision_uart_publish_sample(void)
{
    uint32_t raw_position =
        vision_uart_read_u32_le(VISION_POSITION_OFFSET);
    uint32_t raw_velocity =
        vision_uart_read_u32_le(VISION_VELOCITY_OFFSET);
    uint32_t now_ms;
    uint32_t dt_ms;
    float position_cm;
    float velocity_cm_s;
    float position_mm;
    float previous_position_mm;
    float derived_velocity_mm_s;

    /*
     * The protocol payload is IEEE-754 float32 in little-endian order.
     * Reject the entire sample if either value is NaN or infinity.
     */
    if ((raw_position & FLOAT32_EXPONENT_MASK) ==
            FLOAT32_EXPONENT_MASK ||
        (raw_velocity & FLOAT32_EXPONENT_MASK) ==
            FLOAT32_EXPONENT_MASK)
    {
        vision_uart_error_count++;
        return;
    }

    memcpy(&position_cm, &raw_position, sizeof(position_cm));
    memcpy(&velocity_cm_s, &raw_velocity, sizeof(velocity_cm_s));
    now_ms = (uint32_t)tick_ms;
    position_mm = position_cm * 10.0f;

    if (vision_filter_initialized == 0U)
    {
        vision_filter_initialized = 1U;
        vision_filtered_position_mm = position_mm;
        vision_filtered_velocity_mm_s = velocity_cm_s * 10.0f;
    }
    else
    {
        previous_position_mm = vision_filtered_position_mm;
        vision_filtered_position_mm +=
            VISION_POSITION_FILTER_ALPHA *
            (position_mm - vision_filtered_position_mm);
        dt_ms = now_ms - vision_previous_sample_ms;
        if (dt_ms >= VISION_FILTER_MIN_DT_MS &&
            dt_ms <= VISION_FILTER_MAX_DT_MS)
        {
            derived_velocity_mm_s =
                (vision_filtered_position_mm -
                 previous_position_mm) *
                1000.0f / (float)dt_ms;
            vision_filtered_velocity_mm_s +=
                VISION_VELOCITY_FILTER_ALPHA *
                (derived_velocity_mm_s -
                 vision_filtered_velocity_mm_s);
        }
        else if (dt_ms > VISION_FILTER_MAX_DT_MS)
        {
            vision_filtered_velocity_mm_s =
                velocity_cm_s * 10.0f;
        }
    }

    vision_previous_sample_ms = now_ms;
    vision_ball_position_mm = vision_filtered_position_mm;
    vision_ball_velocity_mm_s = vision_filtered_velocity_mm_s;
    vision_ball_last_update_ms = now_ms;
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
        vision_uart_publish_sample();
    }
}

void vision_uart_reset(void)
{
    vision_ball_position_mm = 0.0f;
    vision_ball_velocity_mm_s = 0.0f;
    vision_ball_frame_count = 0U;
    vision_ball_last_update_ms = 0U;
    vision_uart_error_count = 0U;
    vision_ball_position_valid = 0U;
    vision_last_byte_ms = (uint32_t)tick_ms;
    vision_filter_initialized = 0U;
    vision_filtered_position_mm = 0.0f;
    vision_filtered_velocity_mm_s = 0.0f;
    vision_previous_sample_ms = (uint32_t)tick_ms;
    memset(vision_payload, 0, sizeof(vision_payload));
    vision_uart_reset_parser();
}

static void servo_uart_clear_parser(void)
{
    servo_uart_rx_length = 0U;
    servo_uart_command_ready = 0U;
    memset(servo_uart_rx_line, 0, sizeof(servo_uart_rx_line));
    memset(servo_uart_command, 0, sizeof(servo_uart_command));
}

void control_uart_reset(void)
{
    control_uart_mode = CONTROL_UART_DISABLED;
    control_uart_command_count = 0U;
    control_uart_error_count = 0U;
    servo_uart_clear_parser();
}

void control_uart_set_mode(control_uart_mode_t mode)
{
    control_uart_mode_t previous_mode = control_uart_mode;

    if (mode > CONTROL_UART_OPEN_LOOP_TUNING)
    {
        mode = CONTROL_UART_DISABLED;
    }
    if (mode == previous_mode)
    {
        return;
    }

    control_uart_mode = mode;
    control_uart_command_count = 0U;
    control_uart_error_count = 0U;
    servo_uart_clear_parser();
    if (previous_mode == CONTROL_UART_SERVO_ADJUST ||
        mode == CONTROL_UART_SERVO_ADJUST)
    {
        servo_set_pulse_us(SERVO_NEUTRAL_PULSE_US);
    }
}

static void servo_uart_process_byte(uint8_t byte)
{
    uint8_t index;

    if (control_uart_mode == CONTROL_UART_DISABLED)
    {
        return;
    }

    if (byte == '\r' || byte == '\n')
    {
        if (servo_uart_rx_length == 0U)
        {
            return;
        }
        if (servo_uart_command_ready != 0U)
        {
            control_uart_error_count++;
            servo_uart_rx_length = 0U;
            return;
        }

        for (index = 0U; index < servo_uart_rx_length; index++)
        {
            servo_uart_command[index] = servo_uart_rx_line[index];
        }
        servo_uart_command[servo_uart_rx_length] = '\0';
        servo_uart_rx_length = 0U;
        servo_uart_command_ready = 1U;
        return;
    }

    if (byte == '\b' || byte == 0x7FU)
    {
        if (servo_uart_rx_length > 0U)
        {
            servo_uart_rx_length--;
        }
        return;
    }

    if (byte < 0x20U || byte > 0x7EU)
    {
        control_uart_error_count++;
        return;
    }

    if (servo_uart_rx_length >= SERVO_UART_LINE_SIZE - 1U)
    {
        control_uart_error_count++;
        servo_uart_rx_length = 0U;
        return;
    }

    servo_uart_rx_line[servo_uart_rx_length] = (char)byte;
    servo_uart_rx_length++;
}

static uint8_t servo_uart_parse_pulse(
    const char *text,
    uint16_t *pulse_us)
{
    uint32_t value = 0U;
    uint8_t have_digit = 0U;

    while (*text == ' ' || *text == '=')
    {
        text++;
    }
    if (*text == 'P' || *text == 'p')
    {
        text++;
    }
    while (*text == ' ' || *text == '=')
    {
        text++;
    }

    while (*text >= '0' && *text <= '9')
    {
        have_digit = 1U;
        value = value * 10U + (uint32_t)(*text - '0');
        if (value > 9999U)
        {
            return 0U;
        }
        text++;
    }
    while (*text == ' ')
    {
        text++;
    }

    if (have_digit == 0U || *text != '\0')
    {
        return 0U;
    }
    if (value < SERVO_CONTROL_MIN_PULSE_US ||
        value > SERVO_CONTROL_MAX_PULSE_US)
    {
        return 0U;
    }

    *pulse_us = (uint16_t)value;
    return 1U;
}

static void servo_uart_apply_step(int32_t step_us)
{
    int32_t pulse_us = (int32_t)servo_get_pulse_us() + step_us;

    if (pulse_us < (int32_t)SERVO_CONTROL_MIN_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MIN_PULSE_US;
    }
    else if (pulse_us > (int32_t)SERVO_CONTROL_MAX_PULSE_US)
    {
        pulse_us = SERVO_CONTROL_MAX_PULSE_US;
    }
    servo_set_pulse_us((uint16_t)pulse_us);
}

static uint8_t control_uart_parse_gain(
    const char *text,
    float *gain)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint32_t divisor = 1U;
    uint8_t have_digit = 0U;
    float sign = 1.0f;

    while (*text == ' ' || *text == '=')
    {
        text++;
    }
    if (*text == '-' || *text == '+')
    {
        if (*text == '-')
        {
            sign = -1.0f;
        }
        text++;
    }

    while (*text >= '0' && *text <= '9')
    {
        have_digit = 1U;
        whole = whole * 10U + (uint32_t)(*text - '0');
        if (whole > 999U)
        {
            return 0U;
        }
        text++;
    }

    if (*text == '.')
    {
        text++;
        while (*text >= '0' && *text <= '9')
        {
            if (divisor >= 1000U)
            {
                return 0U;
            }
            fraction = fraction * 10U + (uint32_t)(*text - '0');
            divisor *= 10U;
            have_digit = 1U;
            text++;
        }
    }

    while (*text == ' ')
    {
        text++;
    }
    if (have_digit == 0U || *text != '\0')
    {
        return 0U;
    }

    *gain = sign *
        ((float)whole + (float)fraction / (float)divisor);
    return 1U;
}

static void control_uart_print_pid(void)
{
    uint32_t position_kp =
        (uint32_t)(ball_balance_position_kp * 1000.0f + 0.5f);
    uint32_t position_ki =
        (uint32_t)(ball_balance_position_ki * 1000.0f + 0.5f);
    uint32_t velocity_kp =
        (uint32_t)(ball_balance_velocity_kp * 1000.0f + 0.5f);
    uint32_t velocity_ki =
        (uint32_t)(ball_balance_velocity_ki * 1000.0f + 0.5f);
    uint32_t velocity_limit =
        (uint32_t)(ball_balance_velocity_limit_mm_s + 0.5f);

    printf(
        "PKP=%u.%03u PKI=%u.%03u VKP=%u.%03u VKI=%u.%03u VMAX=%u mm/s\r\n",
        (unsigned int)(position_kp / 1000U),
        (unsigned int)(position_kp % 1000U),
        (unsigned int)(position_ki / 1000U),
        (unsigned int)(position_ki % 1000U),
        (unsigned int)(velocity_kp / 1000U),
        (unsigned int)(velocity_kp % 1000U),
        (unsigned int)(velocity_ki / 1000U),
        (unsigned int)(velocity_ki % 1000U),
        (unsigned int)velocity_limit);
}

static uint8_t control_uart_apply_pid_command(const char *command)
{
    float gain;
    float position_kp = ball_balance_position_kp;
    float position_ki = ball_balance_position_ki;
    float velocity_kp = ball_balance_velocity_kp;
    float velocity_ki = ball_balance_velocity_ki;
    float velocity_limit = ball_balance_velocity_limit_mm_s;

    if (strcmp(command, "?") == 0 ||
        strcmp(command, "PID?") == 0)
    {
        control_uart_print_pid();
        return 1U;
    }
    if (strcmp(command, "PID RESET") == 0)
    {
        ball_balance_reset_pid_gains();
        control_uart_print_pid();
        return 1U;
    }

    if (strncmp(command, "PKP", 3U) == 0)
    {
        if (control_uart_parse_gain(command + 3, &gain) == 0U)
        {
            return 0U;
        }
        position_kp = gain;
    }
    else if (strncmp(command, "PKI", 3U) == 0)
    {
        if (control_uart_parse_gain(command + 3, &gain) == 0U)
        {
            return 0U;
        }
        position_ki = gain;
    }
    else if (strncmp(command, "VKP", 3U) == 0)
    {
        if (control_uart_parse_gain(command + 3, &gain) == 0U)
        {
            return 0U;
        }
        velocity_kp = gain;
    }
    else if (strncmp(command, "VKI", 3U) == 0)
    {
        if (control_uart_parse_gain(command + 3, &gain) == 0U)
        {
            return 0U;
        }
        velocity_ki = gain;
    }
    else if (strncmp(command, "VMAX", 4U) == 0)
    {
        if (control_uart_parse_gain(command + 4, &gain) == 0U)
        {
            return 0U;
        }
        velocity_limit = gain;
    }
    else
    {
        return 0U;
    }

    if (ball_balance_set_cascade_gains(
            position_kp,
            position_ki,
            velocity_kp,
            velocity_ki,
            velocity_limit) == 0U)
    {
        return 0U;
    }
    control_uart_print_pid();
    return 1U;
}

static void control_uart_print_open_loop(void)
{
    printf(
        "OL UP=%u DOWN=%u PSW10=%d NSW10=%d HD=%u DB10=%d VTH10=%d\r\n",
        (unsigned int)ball_static_up_pulse_us,
        (unsigned int)ball_static_down_pulse_us,
        (int)(ball_static_positive_switch_mm * 10.0f),
        (int)(ball_static_negative_switch_mm * 10.0f),
        (unsigned int)ball_static_hold_delta_us,
        (int)(ball_static_hold_deadband_mm * 10.0f),
        (int)(ball_static_velocity_threshold_mm_s * 10.0f));
}

static uint8_t control_uart_apply_open_loop_command(
    const char *command)
{
    float value;
    uint16_t pulse_us;
    uint16_t up_pulse_us = ball_static_up_pulse_us;
    uint16_t down_pulse_us = ball_static_down_pulse_us;
    uint16_t hold_delta_us = ball_static_hold_delta_us;
    float positive_switch_mm =
        ball_static_positive_switch_mm;
    float negative_switch_mm =
        ball_static_negative_switch_mm;
    float hold_deadband_mm =
        ball_static_hold_deadband_mm;
    float velocity_threshold_mm_s =
        ball_static_velocity_threshold_mm_s;

    if (strcmp(command, "?") == 0 ||
        strcmp(command, "OL?") == 0)
    {
        control_uart_print_open_loop();
        return 1U;
    }
    if (strcmp(command, "OL RESET") == 0)
    {
        ball_static_reset_open_loop_config();
        control_uart_print_open_loop();
        return 1U;
    }

    if (strncmp(command, "UP", 2U) == 0)
    {
        if (servo_uart_parse_pulse(
                command + 2, &pulse_us) == 0U)
        {
            return 0U;
        }
        up_pulse_us = pulse_us;
    }
    else if (strncmp(command, "DOWN", 4U) == 0)
    {
        if (servo_uart_parse_pulse(
                command + 4, &pulse_us) == 0U)
        {
            return 0U;
        }
        down_pulse_us = pulse_us;
    }
    else if (strncmp(command, "PSW", 3U) == 0)
    {
        if (control_uart_parse_gain(
                command + 3, &value) == 0U)
        {
            return 0U;
        }
        positive_switch_mm = value;
    }
    else if (strncmp(command, "NSW", 3U) == 0)
    {
        if (control_uart_parse_gain(
                command + 3, &value) == 0U)
        {
            return 0U;
        }
        negative_switch_mm = value;
    }
    else if (strncmp(command, "HD", 2U) == 0)
    {
        if (control_uart_parse_gain(
                command + 2, &value) == 0U ||
            value < 0.0f)
        {
            return 0U;
        }
        hold_delta_us = (uint16_t)(value + 0.5f);
    }
    else if (strncmp(command, "DB", 2U) == 0)
    {
        if (control_uart_parse_gain(
                command + 2, &value) == 0U)
        {
            return 0U;
        }
        hold_deadband_mm = value;
    }
    else if (strncmp(command, "VTH", 3U) == 0)
    {
        if (control_uart_parse_gain(
                command + 3, &value) == 0U)
        {
            return 0U;
        }
        velocity_threshold_mm_s = value;
    }
    else
    {
        return 0U;
    }

    if (ball_static_set_open_loop_config(
            up_pulse_us,
            down_pulse_us,
            positive_switch_mm,
            negative_switch_mm,
            hold_delta_us,
            hold_deadband_mm,
            velocity_threshold_mm_s) == 0U)
    {
        return 0U;
    }
    control_uart_print_open_loop();
    return 1U;
}

void control_uart_service(void)
{
    char command[SERVO_UART_LINE_SIZE];
    uint8_t index;
    uint16_t pulse_us;
    control_uart_mode_t mode = control_uart_mode;

    if (mode == CONTROL_UART_DISABLED ||
        servo_uart_command_ready == 0U)
    {
        return;
    }

    for (index = 0U; index < SERVO_UART_LINE_SIZE; index++)
    {
        command[index] = servo_uart_command[index];
        if (command[index] >= 'a' && command[index] <= 'z')
        {
            command[index] =
                (char)(command[index] - 'a' + 'A');
        }
        if (command[index] == '\0')
        {
            break;
        }
    }
    command[SERVO_UART_LINE_SIZE - 1U] = '\0';
    servo_uart_command_ready = 0U;

    if (mode == CONTROL_UART_PID_TUNING)
    {
        if (control_uart_apply_pid_command(command) == 0U)
        {
            control_uart_error_count++;
            printf(
                "ERR PKP0..20 PKI0..5 VKP0..20 VKI0..10 VMAX10..500\r\n");
            return;
        }
        control_uart_command_count++;
        return;
    }
    if (mode == CONTROL_UART_OPEN_LOOP_TUNING)
    {
        if (control_uart_apply_open_loop_command(command) == 0U &&
            control_uart_apply_pid_command(command) == 0U)
        {
            control_uart_error_count++;
            printf(
                "ERR use OL? or PID? for hybrid tuning help\r\n");
            return;
        }
        control_uart_command_count++;
        return;
    }

    if (strcmp(command, "+") == 0)
    {
        servo_uart_apply_step((int32_t)SERVO_EFFECTIVE_STEP_US);
    }
    else if (strcmp(command, "-") == 0)
    {
        servo_uart_apply_step(-(int32_t)SERVO_EFFECTIVE_STEP_US);
    }
    else if (strcmp(command, "?") == 0 ||
             strcmp(command, "P?") == 0)
    {
        printf("SERVO=%u us\r\n", (unsigned int)servo_get_pulse_us());
        control_uart_command_count++;
        return;
    }
    else if (servo_uart_parse_pulse(command, &pulse_us) != 0U)
    {
        servo_set_pulse_us(pulse_us);
    }
    else
    {
        control_uart_error_count++;
        printf(
            "ERR use P966..P1816, +, -, or ?\r\n");
        return;
    }

    control_uart_command_count++;
    printf("SERVO=%u us\r\n", (unsigned int)servo_get_pulse_us());
}

void UART_0_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_index;

    do
    {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(UART_0_INST);

        switch (interrupt_index)
        {
            case DL_UART_MAIN_IIDX_RX:
            case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
                while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))
                {
                    servo_uart_process_byte(
                        (uint8_t)DL_UART_Main_receiveData(UART_0_INST));
                }
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                control_uart_error_count++;
                servo_uart_rx_length = 0U;
                while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST))
                {
                    (void)DL_UART_Main_receiveData(UART_0_INST);
                }
                break;

            default:
                break;
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
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

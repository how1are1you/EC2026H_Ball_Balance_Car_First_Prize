#include "encoder_calibration.h"

#include <math.h>
#include <stdio.h>

volatile EncoderCalibrationState encoder_calibration_state =
    ENCODER_CALIBRATION_IDLE;
volatile EncoderCalibrationMode encoder_calibration_mode =
    ENCODER_CALIBRATION_MANUAL;
volatile int32_t encoder_calibration_left_count;
volatile int32_t encoder_calibration_right_count;
volatile uint32_t encoder_calibration_left_counts_per_meter;
volatile uint32_t encoder_calibration_right_counts_per_meter;
volatile uint32_t encoder_calibration_left_distance_mm;
volatile uint32_t encoder_calibration_right_distance_mm;
volatile uint8_t encoder_calibration_auto_fault;

static float encoder_calibration_auto_speed;
static uint16_t encoder_calibration_auto_still_ticks;
static uint16_t encoder_calibration_auto_elapsed_ticks;

static uint32_t encoder_calibration_abs_count(int32_t count)
{
    return (count < 0) ? (uint32_t)(-count) : (uint32_t)count;
}

static uint32_t encoder_calibration_abs_difference(
    uint32_t first,
    uint32_t second)
{
    return (first > second) ? (first - second) : (second - first);
}

static uint32_t encoder_calibration_calculate_counts_per_meter(int32_t count)
{
    uint32_t absolute_count = encoder_calibration_abs_count(count);

    return (absolute_count * 1000U +
            ENCODER_CALIBRATION_DISTANCE_MM / 2U) /
           ENCODER_CALIBRATION_DISTANCE_MM;
}

static uint32_t encoder_calibration_calculate_perimeter_um(
    uint32_t counts_per_meter)
{
    if (counts_per_meter == 0U)
    {
        return 0U;
    }

    return (ENCODER_CALIBRATION_COUNTS_PER_REV * 1000000U +
            counts_per_meter / 2U) /
           counts_per_meter;
}

static float encoder_calibration_limit_float(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static void encoder_calibration_update_distances(void)
{
    encoder_calibration_left_distance_mm =
        (uint32_t)(encoder_calibration_abs_count(
                       encoder_calibration_left_count) *
                       1000.0f /
                       ENCODER_LEFT_COUNTS_PER_METER +
                   0.5f);
    encoder_calibration_right_distance_mm =
        (uint32_t)(encoder_calibration_abs_count(
                       encoder_calibration_right_count) *
                       1000.0f /
                       ENCODER_RIGHT_COUNTS_PER_METER +
                   0.5f);
}

static void encoder_calibration_finish_auto(uint8_t fault)
{
    encoder_calibration_auto_speed = 0.0f;
    encoder_calibration_auto_fault = fault;
    encoder_calibration_state = ENCODER_CALIBRATION_DONE;
}

void encoder_calibration_reset(void)
{
    encoder_calibration_state = ENCODER_CALIBRATION_IDLE;
    encoder_calibration_mode = ENCODER_CALIBRATION_MANUAL;
    encoder_calibration_left_count = 0;
    encoder_calibration_right_count = 0;
    encoder_calibration_left_counts_per_meter = 0U;
    encoder_calibration_right_counts_per_meter = 0U;
    encoder_calibration_left_distance_mm = 0U;
    encoder_calibration_right_distance_mm = 0U;
    encoder_calibration_auto_fault = ENCODER_AUTO_FAULT_NONE;
    encoder_calibration_auto_speed = 0.0f;
    encoder_calibration_auto_still_ticks = 0U;
    encoder_calibration_auto_elapsed_ticks = 0U;
}

void encoder_calibration_start(void)
{
    encoder_calibration_reset();
    encoder_calibration_mode = ENCODER_CALIBRATION_MANUAL;
    encoder_calibration_state = ENCODER_CALIBRATION_RUNNING;
}

void encoder_calibration_stop(void)
{
    if (encoder_calibration_state != ENCODER_CALIBRATION_RUNNING)
    {
        return;
    }

    encoder_calibration_left_counts_per_meter =
        encoder_calibration_calculate_counts_per_meter(
            encoder_calibration_left_count);
    encoder_calibration_right_counts_per_meter =
        encoder_calibration_calculate_counts_per_meter(
            encoder_calibration_right_count);
    encoder_calibration_state = ENCODER_CALIBRATION_DONE;
}

void encoder_calibration_start_auto(void)
{
    encoder_calibration_reset();
    encoder_calibration_mode = ENCODER_CALIBRATION_AUTO;
    encoder_calibration_state = ENCODER_CALIBRATION_RUNNING;
}

void encoder_calibration_stop_auto(void)
{
    if (encoder_calibration_mode == ENCODER_CALIBRATION_AUTO &&
        (encoder_calibration_state == ENCODER_CALIBRATION_RUNNING ||
         encoder_calibration_state == ENCODER_CALIBRATION_SETTLING))
    {
        encoder_calibration_finish_auto(ENCODER_AUTO_FAULT_NONE);
    }
}

void encoder_calibration_update(int encoder_count_a, int encoder_count_b)
{
    if (encoder_calibration_state != ENCODER_CALIBRATION_RUNNING &&
        encoder_calibration_state != ENCODER_CALIBRATION_SETTLING)
    {
        return;
    }

    /*
     * Keep the same forward-direction convention used by
     * Get_Velocity_From_Encoder(): forward travel is positive on both sides.
     */
    encoder_calibration_left_count -= encoder_count_a;
    encoder_calibration_right_count += encoder_count_b;
    encoder_calibration_update_distances();

    if (encoder_calibration_mode == ENCODER_CALIBRATION_AUTO &&
        encoder_calibration_state == ENCODER_CALIBRATION_SETTLING)
    {
        if (encoder_count_a == 0 && encoder_count_b == 0)
        {
            encoder_calibration_auto_still_ticks++;
            if (encoder_calibration_auto_still_ticks >=
                ENCODER_AUTO_STILL_TICKS)
            {
                encoder_calibration_finish_auto(ENCODER_AUTO_FAULT_NONE);
            }
        }
        else
        {
            encoder_calibration_auto_still_ticks = 0U;
        }
    }
}

uint8_t encoder_calibration_auto_control(
    float *left_target_speed,
    float *right_target_speed)
{
    float center_distance_mm;
    float remaining_distance_m;
    float desired_speed;
    float braking_speed;
    float distance_error_m;
    float correction;
    float correction_limit;
    const float speed_step =
        ENCODER_AUTO_ACCELERATION_MPS2 * 0.005f;

    *left_target_speed = 0.0f;
    *right_target_speed = 0.0f;

    if (encoder_calibration_mode != ENCODER_CALIBRATION_AUTO)
    {
        return 0U;
    }

    if (encoder_calibration_state == ENCODER_CALIBRATION_DONE ||
        encoder_calibration_state == ENCODER_CALIBRATION_IDLE)
    {
        return 0U;
    }

    encoder_calibration_auto_elapsed_ticks++;
    if (encoder_calibration_auto_elapsed_ticks >= ENCODER_AUTO_TIMEOUT_TICKS)
    {
        encoder_calibration_finish_auto(ENCODER_AUTO_FAULT_TIMEOUT);
        return 0U;
    }

    center_distance_mm =
        0.5f * ((float)encoder_calibration_left_distance_mm +
                (float)encoder_calibration_right_distance_mm);

    if (center_distance_mm > 200.0f &&
        encoder_calibration_abs_difference(
            encoder_calibration_left_distance_mm,
            encoder_calibration_right_distance_mm) >
            ENCODER_AUTO_MAX_DISTANCE_MISMATCH_MM)
    {
        encoder_calibration_finish_auto(ENCODER_AUTO_FAULT_MISMATCH);
        return 0U;
    }

    if (encoder_calibration_state == ENCODER_CALIBRATION_SETTLING)
    {
        return 1U;
    }

    if (center_distance_mm >=
        (float)(ENCODER_AUTO_DISTANCE_MM -
                ENCODER_AUTO_STOP_TOLERANCE_MM))
    {
        encoder_calibration_auto_speed = 0.0f;
        encoder_calibration_auto_still_ticks = 0U;
        encoder_calibration_state = ENCODER_CALIBRATION_SETTLING;
        return 1U;
    }

    remaining_distance_m =
        ((float)ENCODER_AUTO_DISTANCE_MM - center_distance_mm) / 1000.0f;
    braking_speed = sqrtf(
        2.0f * ENCODER_AUTO_ACCELERATION_MPS2 * remaining_distance_m);
    desired_speed = ENCODER_AUTO_CRUISE_SPEED_MPS;
    if (braking_speed < desired_speed)
    {
        desired_speed = braking_speed;
    }

    if (encoder_calibration_auto_speed < desired_speed - speed_step)
    {
        encoder_calibration_auto_speed += speed_step;
    }
    else if (encoder_calibration_auto_speed > desired_speed + speed_step)
    {
        encoder_calibration_auto_speed -= speed_step;
    }
    else
    {
        encoder_calibration_auto_speed = desired_speed;
    }

    distance_error_m =
        ((float)encoder_calibration_left_distance_mm -
         (float)encoder_calibration_right_distance_mm) /
        1000.0f;
    correction = ENCODER_AUTO_SYNC_KP * distance_error_m;
    correction_limit = 0.5f * encoder_calibration_auto_speed;
    if (correction_limit > ENCODER_AUTO_MAX_SYNC_CORRECTION_MPS)
    {
        correction_limit = ENCODER_AUTO_MAX_SYNC_CORRECTION_MPS;
    }
    correction = encoder_calibration_limit_float(
        correction,
        -correction_limit,
        correction_limit);

    *left_target_speed = encoder_calibration_auto_speed - correction;
    *right_target_speed = encoder_calibration_auto_speed + correction;

    return 1U;
}

void encoder_calibration_service(void)
{
    static EncoderCalibrationState last_state = ENCODER_CALIBRATION_IDLE;
    static EncoderCalibrationMode last_mode = ENCODER_CALIBRATION_MANUAL;
    EncoderCalibrationState current_state = encoder_calibration_state;
    EncoderCalibrationMode current_mode = encoder_calibration_mode;

    if (current_state == last_state && current_mode == last_mode)
    {
        return;
    }

    last_state = current_state;
    last_mode = current_mode;

    if (current_state == ENCODER_CALIBRATION_RUNNING)
    {
        if (current_mode == ENCODER_CALIBRATION_AUTO)
        {
            printf("\r\n[AUTO_4M] START target=%u mm speed=%u mm/s\r\n",
                   ENCODER_AUTO_DISTANCE_MM,
                   (unsigned int)(
                       ENCODER_AUTO_CRUISE_SPEED_MPS * 1000.0f));
            printf("[AUTO_4M] Single-click performs an emergency stop.\r\n");
        }
        else
        {
            printf("\r\n[ENC_CAL] START distance=%u mm\r\n",
                   ENCODER_CALIBRATION_DISTANCE_MM);
            printf("[ENC_CAL] Push the car straight, then single-click at the end.\r\n");
        }
    }
    else if (current_state == ENCODER_CALIBRATION_SETTLING)
    {
        printf("[AUTO_4M] BRAKING near target.\r\n");
    }
    else if (current_state == ENCODER_CALIBRATION_DONE)
    {
        if (current_mode == ENCODER_CALIBRATION_AUTO)
        {
            printf("[AUTO_4M] DONE fault=%u L=%ld R=%ld counts\r\n",
                   encoder_calibration_auto_fault,
                   (long)encoder_calibration_left_count,
                   (long)encoder_calibration_right_count);
            printf("[AUTO_4M] ESTIMATED_DISTANCE L=%lu R=%lu mm\r\n",
                   (unsigned long)encoder_calibration_left_distance_mm,
                   (unsigned long)encoder_calibration_right_distance_mm);
            printf("[AUTO_4M] Measure the actual travel with the same body reference point.\r\n");
        }
        else
        {
            uint32_t left_perimeter_um =
                encoder_calibration_calculate_perimeter_um(
                    encoder_calibration_left_counts_per_meter);
            uint32_t right_perimeter_um =
                encoder_calibration_calculate_perimeter_um(
                    encoder_calibration_right_counts_per_meter);

            printf("[ENC_CAL] DONE L=%ld R=%ld counts\r\n",
                   (long)encoder_calibration_left_count,
                   (long)encoder_calibration_right_count);
            printf("[ENC_CAL] COUNTS_PER_METER L=%lu R=%lu\r\n",
                   (unsigned long)encoder_calibration_left_counts_per_meter,
                   (unsigned long)encoder_calibration_right_counts_per_meter);
            printf("[ENC_CAL] EFFECTIVE_PERIMETER L=%lu.%03lu R=%lu.%03lu mm\r\n",
                   (unsigned long)(left_perimeter_um / 1000U),
                   (unsigned long)(left_perimeter_um % 1000U),
                   (unsigned long)(right_perimeter_um / 1000U),
                   (unsigned long)(right_perimeter_um % 1000U));
        }
    }
}

/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "show.h"
#include "IR_Module.h"
#include "ball_balance.h"
#include "ball_hold_lap.h"
#include "ball_static_task.h"
#include "imu/imu.h"
#include "servo.h"
#include "straight_turn_test.h"
#include "turn_calibration.h"
#include "uart_callback.h"

#define VISION_UART_LIVE_TIMEOUT_MS (500UL)
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
extern int Servo;

static void oled_show_text(
    uint8_t x,
    uint8_t y,
    const char *text)
{
    OLED_ShowString(x, y, (const uint8_t *)text);
}

static void oled_show_voltage(uint8_t y);

static uint8_t imu_unsigned_digits(uint32_t value)
{
    uint8_t digits = 1U;

    while (value >= 10U)
    {
        value /= 10U;
        digits++;
    }
    return digits;
}

static void imu_show_signed_tenths(
    uint8_t x,
    uint8_t y,
    float value)
{
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint8_t digits;
    uint8_t number_x;
    uint8_t decimal_x;

    if (value > 99999.9f)
    {
        value = 99999.9f;
    }
    else if (value < -99999.9f)
    {
        value = -99999.9f;
    }

    scaled = (int32_t)(value * 10.0f +
                       ((value >= 0.0f) ? 0.5f : -0.5f));

    if (scaled < 0)
    {
        OLED_ShowChar(x, y, '-', 12, 1);
        magnitude = (uint32_t)(-scaled);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 12, 1);
        magnitude = (uint32_t)scaled;
    }

    whole = magnitude / 10U;
    digits = imu_unsigned_digits(whole);
    number_x = (uint8_t)(x + 6U);
    decimal_x = (uint8_t)(number_x + digits * 6U);
    OLED_ShowNumber(number_x, y, whole, digits, 12);
    OLED_ShowChar(decimal_x, y, '.', 12, 1);
    OLED_ShowChar((uint8_t)(decimal_x + 6U), y,
                  (uint8_t)('0' + magnitude % 10U), 12, 1);
}

static uint8_t imu_show_signed_millidegrees(
    uint8_t x,
    uint8_t y,
    float value)
{
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;
    uint8_t digits;
    uint8_t number_x;
    uint8_t decimal_x;

    if (value > 99999.999f)
    {
        value = 99999.999f;
    }
    else if (value < -99999.999f)
    {
        value = -99999.999f;
    }

    scaled = (int32_t)(value * 1000.0f +
                       ((value >= 0.0f) ? 0.5f : -0.5f));
    if (scaled < 0)
    {
        OLED_ShowChar(x, y, '-', 12, 1);
        magnitude = (uint32_t)(-scaled);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 12, 1);
        magnitude = (uint32_t)scaled;
    }

    whole = magnitude / 1000U;
    fraction = magnitude % 1000U;
    digits = imu_unsigned_digits(whole);
    number_x = (uint8_t)(x + 6U);
    decimal_x = (uint8_t)(number_x + digits * 6U);
    OLED_ShowNumber(number_x, y, whole, digits, 12);
    OLED_ShowChar(decimal_x, y, '.', 12, 1);
    OLED_ShowChar((uint8_t)(decimal_x + 6U), y,
                  (uint8_t)('0' + fraction / 100U), 12, 1);
    OLED_ShowChar((uint8_t)(decimal_x + 12U), y,
                  (uint8_t)('0' + (fraction / 10U) % 10U), 12, 1);
    OLED_ShowChar((uint8_t)(decimal_x + 18U), y,
                  (uint8_t)('0' + fraction % 10U), 12, 1);
    return (uint8_t)(decimal_x + 24U);
}

static uint8_t vision_show_position_mm(
    uint8_t x,
    uint8_t y,
    float value)
{
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;
    uint8_t digits;
    uint8_t number_x;
    uint8_t decimal_x;

    if (value > 9999.99f)
    {
        value = 9999.99f;
    }
    else if (value < -9999.99f)
    {
        value = -9999.99f;
    }

    scaled = (int32_t)(value * 100.0f +
                       ((value >= 0.0f) ? 0.5f : -0.5f));
    if (scaled < 0)
    {
        OLED_ShowChar(x, y, '-', 12, 1);
        magnitude = (uint32_t)(-scaled);
    }
    else
    {
        OLED_ShowChar(x, y, '+', 12, 1);
        magnitude = (uint32_t)scaled;
    }

    whole = magnitude / 100U;
    fraction = magnitude % 100U;
    digits = imu_unsigned_digits(whole);
    number_x = (uint8_t)(x + 6U);
    decimal_x = (uint8_t)(number_x + digits * 6U);
    OLED_ShowNumber(number_x, y, whole, digits, 12);
    OLED_ShowChar(decimal_x, y, '.', 12, 1);
    OLED_ShowChar(
        (uint8_t)(decimal_x + 6U),
        y,
        (uint8_t)('0' + fraction / 10U),
        12,
        1);
    OLED_ShowChar(
        (uint8_t)(decimal_x + 12U),
        y,
        (uint8_t)('0' + fraction % 10U),
        12,
        1);
    return (uint8_t)(decimal_x + 18U);
}

static void imu_debug_oled_show(void)
{
    imu_sample_t sample;
    const char *status_text;
    uint8_t yaw_end_x;

    imu_get_snapshot(&sample);
    switch (sample.status)
    {
        case IMU_STATUS_INITIALIZING:
            status_text = "INIT";
            break;
        case IMU_STATUS_READY:
            status_text = "READY";
            break;
        case IMU_STATUS_DEVICE_NOT_FOUND:
            status_text = "NO DEV";
            break;
        case IMU_STATUS_DMP_ERROR:
            status_text = "DMP ERR";
            break;
        case IMU_STATUS_FIFO_ERROR:
            status_text = "FIFO ERR";
            break;
        default:
            status_text = "OFF";
            break;
    }

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(0, 0, "IMU:");
    oled_show_text(26, 0, status_text);
    oled_show_text(82, 0, "ID:");
    OLED_ShowNumber(100, 0, sample.who_am_i, 3, 12);

    oled_show_text(0, 10, "Y:");
    yaw_end_x =
        imu_show_signed_millidegrees(16, 10, sample.yaw_continuous_deg);
    oled_show_text((uint8_t)(yaw_end_x + 4U), 10, "deg");

    oled_show_text(0, 20, "GZ:");
    imu_show_signed_tenths(24, 20, sample.gyro_z_dps);

    oled_show_text(0, 30, "AX:");
    imu_show_signed_tenths(24, 30, sample.accel_g[0]);
    oled_show_text(66, 30, "g");

    oled_show_text(0, 40, "AY:");
    imu_show_signed_tenths(24, 40, sample.accel_g[1]);
    oled_show_text(66, 40, "g");

    oled_show_text(0, 50, "AZ:");
    imu_show_signed_tenths(24, 50, sample.accel_g[2]);
    oled_show_text(66, 50, "g");
    OLED_Refresh_Gram();
}

void imu_startup_oled_show(uint8_t seconds_remaining)
{
    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(12, 4, "IMU CALIBRATE");
    oled_show_text(8, 22, "KEEP CAR STILL");
    oled_show_text(24, 40, "WAIT:");
    OLED_ShowNumber(68, 40, seconds_remaining, 1, 12);
    oled_show_text(80, 40, "SEC");
    OLED_Refresh_Gram();
}

static const char *one_lap_state_text(LF04_LapState_t state)
{
    switch (state)
    {
        case LF04_LAP_IDLE:
            return "IDLE";
        case LF04_LAP_STRAIGHT_1:
            return "S1";
        case LF04_LAP_CURVE_1:
            return "C1";
        case LF04_LAP_STRAIGHT_2:
            return "S2";
        case LF04_LAP_CURVE_2:
            return "C2";
        case LF04_LAP_BRAKING:
            return "BRAKE";
        case LF04_LAP_SETTLING:
            return "SET";
        case LF04_LAP_DONE:
            return "DONE";
        case LF04_LAP_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static void one_lap_oled_show(void)
{
    uint32_t distance_mm =
        (uint32_t)(LF04_LapDistanceM * 1000.0f + 0.5f);
    uint32_t speed_mm_s =
        (uint32_t)(LF04_CommandSpeed * 1000.0f + 0.5f);

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(0, 0, "ONE LAP");
    oled_show_text(58, 0, one_lap_state_text(LF04_LapState));

    oled_show_text(0, 10, "S:");
    OLED_ShowNumber(16, 10, distance_mm, 4, 12);
    oled_show_text(44, 10, "mm");

    oled_show_text(0, 20, "Y:");
    imu_show_signed_tenths(16, 20, LF04_LapYawDeg);
    oled_show_text(64, 20, "deg");

    oled_show_text(0, 30, "IR:");
    OLED_ShowNumber(20, 30, LF04_DH2_State, 1, 12);
    OLED_ShowNumber(28, 30, LF04_DH3_State, 1, 12);
    oled_show_text(46, 30, "E:");
    imu_show_signed_tenths(62, 30, LF04_SteeringError);

    oled_show_text(0, 40, "V:");
    OLED_ShowNumber(16, 40, speed_mm_s, 3, 12);
    oled_show_text(38, 40, "mm/s");
    oled_show_text(80, 40, (LF04_ImuUsed != 0U) ? "IMU" : "ENC");

    oled_show_text(0, 50, "T:");
    OLED_ShowNumber(16, 50, LF04_LapElapsedMs / 1000U, 2, 12);
    oled_show_text(32, 50, "s");
    oled_show_text(52, 50, "F:");
    OLED_ShowNumber(68, 50, LF04_LapFault, 1, 12);
    OLED_Refresh_Gram();
}

static const char *turn_calibration_state_text(
    TurnCalibrationState_t state)
{
    switch (state)
    {
        case TURN_CALIBRATION_IDLE:
            return "IDLE";
        case TURN_CALIBRATION_RUNNING:
            return "RUN";
        case TURN_CALIBRATION_BRAKING:
            return "BRAKE";
        case TURN_CALIBRATION_DONE:
            return "DONE";
        case TURN_CALIBRATION_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static void turn_calibration_oled_show(void)
{
    uint32_t radius_mm =
        (uint32_t)(TurnCalibrationRadiusM * 1000.0f + 0.5f);
    uint32_t distance_mm =
        (uint32_t)(TurnCalibrationDistanceM * 1000.0f + 0.5f);
    uint32_t speed_mm_s =
        (uint32_t)(TurnCalibrationCommandSpeed * 1000.0f + 0.5f);

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(0, 0, "TURN CAL CW");
    oled_show_text(
        76, 0, turn_calibration_state_text(TurnCalibrationState));

    oled_show_text(0, 10, "R:");
    OLED_ShowNumber(16, 10, radius_mm, 3, 12);
    oled_show_text(38, 10, "mm");

    oled_show_text(0, 20, "Y:");
    imu_show_signed_tenths(16, 20, TurnCalibrationYawDeg);
    oled_show_text(64, 20, "deg");

    oled_show_text(0, 30, "D:");
    OLED_ShowNumber(16, 30, distance_mm, 4, 12);
    oled_show_text(44, 30, "mm");

    oled_show_text(0, 40, "V:");
    OLED_ShowNumber(16, 40, speed_mm_s, 3, 12);
    oled_show_text(38, 40, "mm/s");
    oled_show_text(82, 40, "F:");
    OLED_ShowNumber(98, 40, TurnCalibrationFault, 1, 12);

    if (Flag_Stop)
    {
        oled_show_text(0, 50, "1:R+20  2:START");
    }
    else
    {
        oled_show_text(0, 50, "1:STOP");
    }
    OLED_Refresh_Gram();
}

static const char *straight_turn_state_text(
    StraightTurnState_t state)
{
    switch (state)
    {
        case STRAIGHT_TURN_IDLE:
            return "IDLE";
        case STRAIGHT_TURN_STRAIGHT_1:
            return "LINE1";
        case STRAIGHT_TURN_ARC_1:
            return "ARC1";
        case STRAIGHT_TURN_STRAIGHT_2:
            return "LINE2";
        case STRAIGHT_TURN_ARC_2:
            return "ARC2";
        case STRAIGHT_TURN_POST_LAP:
            return "POST1M";
        case STRAIGHT_TURN_BRAKING:
            return "BRAKE";
        case STRAIGHT_TURN_DONE:
            return "DONE";
        case STRAIGHT_TURN_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static const char *straight_turn_status_text(
    StraightTurnState_t state)
{
    switch (state)
    {
        case STRAIGHT_TURN_IDLE:
            return (StraightTurnElapsedMs == 0U) ? "READY" : "STOP";
        case STRAIGHT_TURN_DONE:
            return "DONE";
        case STRAIGHT_TURN_FAULT:
            return "FAULT";
        default:
            return "RUN";
    }
}

static void straight_turn_show_time(uint8_t y)
{
    uint32_t elapsed_ms = StraightTurnElapsedMs;
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t milliseconds = elapsed_ms % 1000U;

    if (seconds > 99U)
    {
        seconds = 99U;
        milliseconds = 999U;
    }

    oled_show_text(0, y, "TIME:");
    OLED_ShowChar(40, y, (uint8_t)('0' + seconds / 10U), 12, 1);
    OLED_ShowChar(46, y, (uint8_t)('0' + seconds % 10U), 12, 1);
    OLED_ShowChar(52, y, '.', 12, 1);
    OLED_ShowChar(
        58, y, (uint8_t)('0' + milliseconds / 100U), 12, 1);
    OLED_ShowChar(
        64, y, (uint8_t)('0' + (milliseconds / 10U) % 10U), 12, 1);
    OLED_ShowChar(
        70, y, (uint8_t)('0' + milliseconds % 10U), 12, 1);
    OLED_ShowChar(78, y, 's', 12, 1);
}

static void straight_turn_oled_show(void)
{
    uint32_t target_speed_mm_s =
        (Run_Mode == RUN_MODE_BALL_LAP) ?
            (uint32_t)(STRAIGHT_TURN_BALL_SPEED_MPS * 1000.0f + 0.5f) :
            (uint32_t)(STRAIGHT_TURN_FAST_SPEED_MPS * 1000.0f + 0.5f);

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(
        0, 0, (Run_Mode == RUN_MODE_BALL_LAP) ? "BALL LAP" : "ONE LAP");
    oled_show_text(
        72, 0, straight_turn_status_text(StraightTurnState));

    straight_turn_show_time(10);

    oled_show_text(0, 20, "STAGE:");
    oled_show_text(
        48, 20, straight_turn_state_text(StraightTurnState));

    oled_show_text(0, 30, "SPD:");
    OLED_ShowNumber(32, 30, target_speed_mm_s, 3, 12);
    oled_show_text(54, 30, "mm/s");

    oled_show_voltage(40);

    if (StraightTurnState == STRAIGHT_TURN_DONE)
    {
        oled_show_text(0, 50, "TIME LOCKED");
    }
    else if (StraightTurnState == STRAIGHT_TURN_FAULT)
    {
        oled_show_text(0, 50, "FAULT:");
        OLED_ShowNumber(48, 50, StraightTurnFault, 1, 12);
        oled_show_text(72, 50, "1:RETRY");
    }
    else
    {
        oled_show_text(0, 50, Flag_Stop ? "1:START" : "1:STOP");
    }
    OLED_Refresh_Gram();
}

static void ball_hold_show_time(
    uint8_t y,
    uint32_t elapsed_ms)
{
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t milliseconds = elapsed_ms % 1000U;

    if (seconds > 99U)
    {
        seconds = 99U;
        milliseconds = 999U;
    }

    oled_show_text(0, y, "TIME:");
    OLED_ShowNumber(34, y, seconds, 2, 12);
    OLED_ShowChar(46, y, '.', 12, 1);
    OLED_ShowNumber(52, y, milliseconds, 3, 12);
    oled_show_text(72, y, "s");
}

static const char *ball_hold_state_text(void)
{
    switch (ball_hold_lap_state)
    {
        case BALL_HOLD_LAP_READY:
            return "READY";
        case BALL_HOLD_LAP_CAPTURING:
            return "CAPTURE";
        case BALL_HOLD_LAP_RUNNING:
            return "RUN";
        case BALL_HOLD_LAP_POST_LAP:
            return "POST1M";
        case BALL_HOLD_LAP_BRAKING:
            return "BRAKE";
        case BALL_HOLD_LAP_DONE:
            return "DONE";
        case BALL_HOLD_LAP_ABORTED:
            return "ABORT";
        case BALL_HOLD_LAP_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static void ball_hold_show_position(
    uint8_t y,
    const char *label,
    float position_mm)
{
    uint8_t end_x;

    oled_show_text(0, y, label);
    end_x = vision_show_position_mm(30, y, position_mm);
    oled_show_text((uint8_t)(end_x + 3U), y, "mm");
}

static void ball_hold_lap_oled_show(void)
{
    uint32_t post_distance_mm;

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(0, 0, "BALL HOLD");
    oled_show_text(66, 0, ball_hold_state_text());

    if (ball_hold_lap_state == BALL_HOLD_LAP_CAPTURING)
    {
        oled_show_text(0, 10, "MS:");
        OLED_ShowNumber(
            24, 10,
            ball_hold_lap_capture_elapsed_ms % 1000U,
            3, 12);
        oled_show_text(54, 10, "N:");
        OLED_ShowNumber(
            66, 10,
            ball_hold_lap_capture_frames % 100U,
            2, 12);
        ball_hold_show_position(
            20, "AVG:", ball_hold_lap_capture_mean_mm);
        ball_hold_show_position(
            30, "RNG:", ball_hold_lap_capture_span_mm);
        if (ball_hold_lap_current_valid != 0U)
        {
            ball_hold_show_position(
                40, "NOW:", ball_hold_lap_current_mm);
        }
        else
        {
            oled_show_text(0, 40, "NOW:NO VISION");
        }
        oled_show_text(0, 50, "1:CANCEL");
        OLED_Refresh_Gram();
        return;
    }

    if (ball_hold_lap_state == BALL_HOLD_LAP_READY)
    {
        if (vision_ball_position_valid != 0U &&
            (uint32_t)(tick_ms -
                       vision_ball_last_update_ms) <=
                BALL_HOLD_LAP_VISION_TIMEOUT_MS)
        {
            ball_hold_show_position(
                20, "POS:", vision_ball_position_mm);
        }
        else
        {
            oled_show_text(0, 20, "POS:NO VISION");
        }
        oled_show_text(0, 40, "STABLE 300ms");
        oled_show_text(0, 50, "1:START");
        OLED_Refresh_Gram();
        return;
    }

    if ((ball_hold_lap_state == BALL_HOLD_LAP_ABORTED ||
         ball_hold_lap_state == BALL_HOLD_LAP_FAULT) &&
        ball_hold_lap_time_ms == 0U)
    {
        oled_show_text(0, 10, "TIME:--");
    }
    else
    {
        ball_hold_show_time(
            10,
            (ball_hold_lap_time_ms != 0U) ?
                ball_hold_lap_time_ms :
                StraightTurnElapsedMs);
    }
    ball_hold_show_position(
        20, "REF:", ball_hold_lap_target_mm);

    if (ball_hold_lap_state == BALL_HOLD_LAP_POST_LAP)
    {
        post_distance_mm =
            (uint32_t)(
                StraightTurnPostLapDistanceM *
                1000.0f + 0.5f);
        oled_show_text(0, 30, "POST:");
        OLED_ShowNumber(36, 30, post_distance_mm, 4, 12);
        oled_show_text(66, 30, "mm");
    }
    else if (ball_hold_lap_current_valid != 0U)
    {
        ball_hold_show_position(
            30, "NOW:", ball_hold_lap_current_mm);
    }
    else
    {
        oled_show_text(0, 30, "NOW:NO VISION");
    }

    ball_hold_show_position(
        40, "MAX:", ball_hold_lap_max_abs_error_mm);

    if (ball_hold_lap_state == BALL_HOLD_LAP_DONE)
    {
        oled_show_text(
            0, 50,
            (ball_hold_lap_time_pass != 0U) ?
                "T:PASS " : "T:OVERTIME ");
        oled_show_text(
            54, 50,
            (ball_hold_lap_position_pass != 0U) ?
                "E:PASS" : "E:FAIL");
    }
    else if (ball_hold_lap_state == BALL_HOLD_LAP_FAULT)
    {
        oled_show_text(0, 50, "FAULT:");
        OLED_ShowNumber(
            42, 50, ball_hold_lap_fault, 1, 12);
    }
    else if (ball_hold_lap_state == BALL_HOLD_LAP_ABORTED)
    {
        oled_show_text(0, 50, "ABORT 1:RETRY");
    }
    else
    {
        oled_show_text(0, 50, "1:STOP");
    }

    OLED_Refresh_Gram();
}

static const char *ball_static_state_text(void)
{
    switch (ball_static_state)
    {
        case BALL_STATIC_READY:
            return (ball_static_ready != 0U) ?
                "READY" : "CENTER";
        case BALL_STATIC_MOVE_POS:
            return "TO+5";
        case BALL_STATIC_HOLD_POS:
            return "HOLD+";
        case BALL_STATIC_MOVE_NEG:
            return "TO-5";
        case BALL_STATIC_HOLD_NEG:
            return "HOLD-";
        case BALL_STATIC_PID_HOLD:
            return "PID-5";
        case BALL_STATIC_DONE:
            return "DONE";
        case BALL_STATIC_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

static void ball_static_show_time(uint8_t y)
{
    uint32_t elapsed_ms = ball_static_elapsed_ms;
    uint32_t seconds = elapsed_ms / 1000U;
    uint32_t milliseconds = elapsed_ms % 1000U;

    if (seconds > 9U)
    {
        seconds = 9U;
        milliseconds = 999U;
    }

    oled_show_text(0, y, "TIME:");
    OLED_ShowNumber(34, y, seconds, 1, 12);
    OLED_ShowChar(40, y, '.', 12, 1);
    OLED_ShowNumber(46, y, milliseconds, 3, 12);
    oled_show_text(66, y, "s");
}

static void ball_static_oled_show(void)
{
    uint8_t position_end_x;
    uint32_t positive_error =
        (uint32_t)(ball_static_positive_max_error_mm + 0.5f);
    uint32_t negative_error =
        (uint32_t)(ball_static_negative_max_error_mm + 0.5f);

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    oled_show_text(0, 0, "STATIC:");
    oled_show_text(48, 0, ball_static_state_text());

    ball_static_show_time(10);

    oled_show_text(0, 20, "POS:");
    if (vision_ball_position_valid != 0U)
    {
        position_end_x = vision_show_position_mm(
            30, 20, vision_ball_position_mm);
        oled_show_text((uint8_t)(position_end_x + 4U), 20, "mm");
    }
    else
    {
        oled_show_text(30, 20, "--");
    }

    oled_show_text(0, 30, "REF:");
    position_end_x =
        vision_show_position_mm(30, 30, ball_static_target_mm);
    oled_show_text((uint8_t)(position_end_x + 4U), 30, "mm");

    oled_show_text(0, 40, "VEL:");
    if (vision_ball_position_valid != 0U)
    {
        position_end_x = vision_show_position_mm(
            30, 40, vision_ball_velocity_mm_s);
        oled_show_text((uint8_t)(position_end_x + 4U), 40, "mm/s");
    }
    else
    {
        oled_show_text(30, 40, "--");
    }

    if (ball_static_state == BALL_STATIC_FAULT)
    {
        oled_show_text(0, 50, "FAULT:");
        OLED_ShowNumber(42, 50, ball_static_fault, 1, 12);
        oled_show_text(54, 50, "1:RESET");
    }
    else if (ball_static_state == BALL_STATIC_DONE)
    {
        oled_show_text(0, 50, "E+:");
        OLED_ShowNumber(18, 50, positive_error, 1, 12);
        oled_show_text(30, 50, "E-:");
        OLED_ShowNumber(48, 50, negative_error, 1, 12);
        oled_show_text(60, 50, "mm");
    }
    else if (ball_static_state == BALL_STATIC_READY)
    {
        oled_show_text(
            0, 50,
            (ball_static_ready != 0U) ?
                "1:START HOLD" :
                "PUT BALL AT O");
    }
    else
    {
        oled_show_text(0, 50, "1:STOP HOLD");
    }
    OLED_Refresh_Gram();
}

static void servo_adjust_oled_show(void)
{
    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

    oled_show_text(0, 0, "SERVO ADJ UART0");
    oled_show_text(0, 10, "PWM:");
    OLED_ShowNumber(30, 10, servo_get_pulse_us(), 4, 12);
    oled_show_text(60, 10, "us");

    oled_show_text(0, 20, "RANGE:");
    OLED_ShowNumber(42, 20, SERVO_CONTROL_MIN_PULSE_US, 4, 12);
    OLED_ShowChar(66, 20, '-', 12, 1);
    OLED_ShowNumber(72, 20, SERVO_CONTROL_MAX_PULSE_US, 4, 12);

    oled_show_text(0, 30, "STEP:+/-10us");
    oled_show_text(0, 40, "CMD:Pxxxx + - ?");
    oled_show_text(0, 50, "OK:");
    OLED_ShowNumber(
        18, 50, control_uart_command_count % 1000U, 3, 12);
    oled_show_text(42, 50, "ERR:");
    OLED_ShowNumber(
        66, 50, control_uart_error_count % 1000U, 3, 12);
    oled_show_text(90, 50, "HOLD");
    OLED_Refresh_Gram();
}

static void menu_show_item(
    uint8_t y,
    uint8_t mode,
    const char *text)
{
    OLED_ShowChar(0, y, (Menu_Selection == mode) ? '>' : ' ', 12, 1);
    oled_show_text(6, y, text);
}

static void oled_show_voltage(uint8_t y)
{
    float voltage = Voltage;
    uint16_t voltage_tenths;

    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }
    else if (voltage > 99.9f)
    {
        voltage = 99.9f;
    }
    voltage_tenths = (uint16_t)(voltage * 10.0f + 0.5f);

    oled_show_text(0, y, "VOLT:");
    OLED_ShowNumber(40, y, voltage_tenths / 10U, 2, 12);
    OLED_ShowChar(52, y, '.', 12, 1);
    OLED_ShowNumber(58, y, voltage_tenths % 10U, 1, 12);
    OLED_ShowChar(64, y, 'V', 12, 1);
}

static void menu_oled_show(void)
{
    uint8_t page =
        (uint8_t)(Menu_SelectionIndex /
                  MENU_ITEMS_PER_PAGE);
    uint8_t first =
        (uint8_t)(page * MENU_ITEMS_PER_PAGE);
    uint8_t end =
        (uint8_t)(first + MENU_ITEMS_PER_PAGE);
    uint8_t page_count =
        (uint8_t)((MENU_MODE_COUNT +
                   MENU_ITEMS_PER_PAGE - 1U) /
                  MENU_ITEMS_PER_PAGE);
    uint8_t index;
    uint8_t y = 0U;

    if (end > MENU_MODE_COUNT)
    {
        end = MENU_MODE_COUNT;
    }

    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

    for (index = first; index < end; index++)
    {
        menu_show_item(
            y,
            Menu_Items[index].mode,
            Menu_Items[index].label);
        y = (uint8_t)(y + 12U);
    }

    OLED_ShowChar(0, 52, 'P', 12, 1);
    OLED_ShowNumber(6, 52, page + 1U, 1, 12);
    OLED_ShowChar(12, 52, '/', 12, 1);
    OLED_ShowNumber(18, 52, page_count, 1, 12);
    oled_show_text(30, 52, "1>N 2>OK");

    OLED_Refresh_Gram();
}

void oled_show(void)
{
    static unsigned long last_refresh_ms;

    if ((unsigned long)(tick_ms - last_refresh_ms) < 50UL)
    {
        return;
    }
    last_refresh_ms = tick_ms;

    if (Menu_Active)
    {
        menu_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_ONE_LAP)
    {
        one_lap_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_IMU_DEBUG)
    {
        imu_debug_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_BALL_STATIC)
    {
        ball_static_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_SERVO_ADJUST)
    {
        servo_adjust_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_BALL_HOLD_LAP)
    {
        ball_hold_lap_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_TURN_CAL)
    {
        turn_calibration_oled_show();
        return;
    }
    if (Run_Mode == RUN_MODE_STRAIGHT_TURN ||
        Run_Mode == RUN_MODE_BALL_LAP)
    {
        straight_turn_oled_show();
        return;
    }

     memset(OLED_GRAM,0, 128*8*sizeof(u8)); //GRAM清零但不立即刷新，防止花屏
        //=============第一行显示小车模式=======================//
	
             if(Car_Mode==0)   OLED_ShowString(0,0,"Mec ");
        else if(Car_Mode==1)   OLED_ShowString(0,0,"Omni");
        else if(Car_Mode==2)   OLED_ShowString(0,0,"AKM ");
        else if(Car_Mode==3)   OLED_ShowString(0,0,"Diff");
        else if(Car_Mode==4)   OLED_ShowString(0,0,"4WD ");
		else if(Car_Mode==5)   OLED_ShowString(0,0,"Tank");
		OLED_ShowString(0,10,"Servo: ");
		OLED_ShowNumber(60,10, myabs((int)(Servo)),4,12);
	
	    if(Run_Mode==0)   OLED_ShowString(90,0,"APP");
		OLED_ShowChar(0,20,'L',12,1);
		OLED_ShowChar(6,20,(MotorA.Motor_Pwm < 0) ? '-' : '+',12,1);
		OLED_ShowNumber(12,20,myabs((int)MotorA.Motor_Pwm),4,12);
		OLED_ShowChar(48,20,'R',12,1);
		OLED_ShowChar(54,20,(MotorB.Motor_Pwm < 0) ? '-' : '+',12,1);
		OLED_ShowNumber(60,20,myabs((int)MotorB.Motor_Pwm),4,12);
		OLED_ShowChar(96,20,'P',12,1);
		OLED_ShowChar(102,20,'W',12,1);
		OLED_ShowChar(108,20,'M',12,1);
        //=============第四行显示左编码器PWM与读数=======================//
                              OLED_ShowString(00,30,"L");
        if((MotorA.Target_Encoder*1000)<0)          OLED_ShowString(16,30,"-"),
                                                  OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);
        if((MotorA.Target_Encoder*1000)>=0)       OLED_ShowString(16,30,"+"),
                              OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);

        if(MotorA.Current_Encoder<0)   OLED_ShowString(60,30,"-");
        if(MotorA.Current_Encoder>=0)    OLED_ShowString(60,30,"+");
                              OLED_ShowNumber(68,30,myabs((int)(MotorA.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,30,"mm/s");

        //=============第五行显示右编码器PWM与读数=======================//
                              OLED_ShowString(00,40,"R");
        if((MotorB.Target_Encoder*1000)<0)         OLED_ShowString(16,40,"-"),
                                                    OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);
        if((MotorB.Target_Encoder*1000)>=0)    		OLED_ShowString(16,40,"+"),
													OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);

        if(MotorB.Current_Encoder<0)    OLED_ShowString(60,40,"-");
        if(MotorB.Current_Encoder>=0)   OLED_ShowString(60,40,"+");
                              OLED_ShowNumber(68,40,myabs((int)(MotorB.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,40,"mm/s");

        //=============第六行显示电压与电机开关=======================//
                              OLED_ShowString(0,50,"V");
                                                    OLED_ShowString(30,50,".");
                                                    OLED_ShowString(64,50,"V");
                                                    OLED_ShowNumber(19,50,(int)Voltage,2,12);
                                                    OLED_ShowNumber(39,50,(u16)(Voltage*10)%10,2,12);
        if(Flag_Stop)         OLED_ShowString(95,50,"OFF");
        if(!Flag_Stop)        OLED_ShowString(95,50,"ON ");

        //=============刷新=======================//
        OLED_Refresh_Gram();
		
}
/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{
  static u8 flag;
    int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;

    if (Menu_Active ||
        Run_Mode == RUN_MODE_ONE_LAP ||
        Run_Mode == RUN_MODE_IMU_DEBUG ||
        Run_Mode == RUN_MODE_TURN_CAL ||
        Run_Mode == RUN_MODE_STRAIGHT_TURN ||
        Run_Mode == RUN_MODE_BALL_LAP ||
        Run_Mode == RUN_MODE_BALL_HOLD_LAP ||
        Run_Mode == RUN_MODE_BALL_STATIC ||
        Run_Mode == RUN_MODE_SERVO_ADJUST)
    {
        return;
    }

    Voltage_Show=(Voltage-1000)*2/3;        if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
    Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;           //对编码器数据就行数据处理便于图形化
    Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
    flag=!flag;
    if(PID_Send==1)         //发送PID参数,在APP调参界面显示
    {
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI,(int)Servo_Init,(int)0,(int)0,0,0,0);//打印到APP上面
        PID_Send=0;
    }
   else if(flag==0)     // 发送电池电压，速度，角度等参数，在APP首页显示
        printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)0); //打印到APP上面
     else                               //发送小车姿态角，在波形界面显示
      printf("{B%d:%d:%d}$",(int)0,(int)0,(int)0); //x，y，z轴角度 在APP上面显示波形
                                                                                                                    //可按格式自行增加显示波形，最多可显示五个
}
/**************************************************************************
Function: Virtual oscilloscope sends data to upper computer
Input   : none
Output  : none
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
    u8 i;//计数变量
    float Vol;                              //电压变量
    unsigned char Send_Count; //串口需要发送的数据个数
 //   Vol=(float)Voltage/100;
    DataScope_Get_Channel_Data( 0, 1 );       //显示角度 单位：度（°）
    DataScope_Get_Channel_Data( 0, 2 );         //显示超声波测量的距离 单位：CM
    DataScope_Get_Channel_Data( 0, 3 );                 //显示电池电压 单位：V
//      DataScope_Get_Channel_Data( 0 , 4 );
//      DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//      DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//      DataScope_Get_Channel_Data(0, 7 );
//      DataScope_Get_Channel_Data( 0, 8 );
//      DataScope_Get_Channel_Data(0, 9 );
//      DataScope_Get_Channel_Data( 0 , 10);
    Send_Count = DataScope_Data_Generate(3);
    for(i = 0 ; i < Send_Count; i++)
    {
//        uart0_send_char(DataScope_OutPut_Buffer[i]);
    }
}


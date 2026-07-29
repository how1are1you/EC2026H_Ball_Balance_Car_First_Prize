#include "imu.h"

#include "imu_bus.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_I2C_ADDRESS (0x68U)
#define IMU_WHO_AM_I_REG (0x75U)
#define IMU_WHO_AM_I_VALUE (0x68U)
#define IMU_SAMPLE_RATE_HZ (200U)
#define IMU_Q30_SCALE (1073741824.0f)
#define IMU_RAD_TO_DEG (57.2957795131f)
#define IMU_MAX_FIFO_PACKETS_PER_SERVICE (8U)

/*
 * Default WHEELTEC mounting: sensor X and Y are opposite the vehicle axes,
 * sensor Z points upward. Change this matrix after the physical mounting is
 * confirmed.
 */
static signed char imu_orientation[9] = {
    -1, 0, 0,
     0,-1, 0,
     0, 0, 1
};

static imu_sample_t imu_published;
static volatile uint8_t imu_initialized;
static volatile uint8_t imu_sample_pending;
static volatile uint8_t imu_zero_yaw_requested;
static volatile uint32_t imu_interrupt_count;
static uint32_t imu_sample_count;
static uint32_t imu_fifo_error_count;
static float imu_gyro_sensitivity = 16.4f;
static float imu_accel_sensitivity = 16384.0f;
static uint8_t imu_yaw_started;
static float imu_last_wrapped_yaw;
static float imu_unwrapped_yaw;
static float imu_yaw_zero;

static float imu_clamp_unit(float value)
{
    if (value > 1.0f)
    {
        return 1.0f;
    }
    if (value < -1.0f)
    {
        return -1.0f;
    }
    return value;
}

static float imu_wrap_180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static unsigned short imu_orientation_row_to_scale(
    signed char const *row)
{
    if (row[0] > 0)
    {
        return 0U;
    }
    if (row[0] < 0)
    {
        return 4U;
    }
    if (row[1] > 0)
    {
        return 1U;
    }
    if (row[1] < 0)
    {
        return 5U;
    }
    if (row[2] > 0)
    {
        return 2U;
    }
    if (row[2] < 0)
    {
        return 6U;
    }
    return 7U;
}

static unsigned short imu_orientation_to_scalar(
    signed char const *matrix)
{
    unsigned short scalar;

    scalar = imu_orientation_row_to_scale(matrix);
    scalar |= (unsigned short)
        (imu_orientation_row_to_scale(matrix + 3) << 3);
    scalar |= (unsigned short)
        (imu_orientation_row_to_scale(matrix + 6) << 6);
    return scalar;
}

static void imu_publish(imu_sample_t const *sample)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    imu_published = *sample;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void imu_publish_status(imu_status_t status, uint8_t who_am_i)
{
    imu_sample_t sample;

    imu_get_snapshot(&sample);
    sample.status = status;
    sample.who_am_i = who_am_i;
    sample.interrupt_count = imu_interrupt_count;
    sample.sample_count = imu_sample_count;
    sample.fifo_error_count = imu_fifo_error_count;
    imu_publish(&sample);
}

static int imu_dmp_start(void)
{
    unsigned short accel_sensitivity;
    unsigned short features =
        DMP_FEATURE_6X_LP_QUAT |
        DMP_FEATURE_SEND_RAW_ACCEL |
        DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;

    if (mpu_init() != 0)
    {
        return -1;
    }
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
    {
        return -2;
    }
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
    {
        return -3;
    }
    if (mpu_set_sample_rate(IMU_SAMPLE_RATE_HZ) != 0)
    {
        return -4;
    }
    if (dmp_load_motion_driver_firmware() != 0)
    {
        return -5;
    }
    if (dmp_set_orientation(
            imu_orientation_to_scalar(imu_orientation)) != 0)
    {
        return -6;
    }
    if (dmp_enable_feature(features) != 0)
    {
        return -7;
    }
    if (dmp_set_fifo_rate(IMU_SAMPLE_RATE_HZ) != 0)
    {
        return -8;
    }
    if (mpu_get_gyro_sens(&imu_gyro_sensitivity) != 0)
    {
        return -9;
    }
    if (mpu_get_accel_sens(&accel_sensitivity) == 0)
    {
        imu_accel_sensitivity = (float)accel_sensitivity;
    }
    else
    {
        /*
         * The vendor API returns an integer sensitivity. Keep the known
         * +/-2 g default if this optional query is unavailable.
         */
        imu_accel_sensitivity = 16384.0f;
    }
    if (mpu_set_dmp_state(1U) != 0)
    {
        return -10;
    }
    return 0;
}

int imu_init(void)
{
    uint8_t who_am_i = 0U;
    int result;

    memset(&imu_published, 0, sizeof(imu_published));
    imu_published.status = IMU_STATUS_INITIALIZING;
    imu_initialized = 0U;
    imu_sample_pending = 0U;
    imu_zero_yaw_requested = 0U;
    imu_interrupt_count = 0U;
    imu_sample_count = 0U;
    imu_fifo_error_count = 0U;
    imu_yaw_started = 0U;
    imu_last_wrapped_yaw = 0.0f;
    imu_unwrapped_yaw = 0.0f;
    imu_yaw_zero = 0.0f;

    imu_bus_init();
    if (imu_bus_read(IMU_I2C_ADDRESS, IMU_WHO_AM_I_REG,
                     1U, &who_am_i) != 0 ||
        who_am_i != IMU_WHO_AM_I_VALUE)
    {
        imu_publish_status(IMU_STATUS_DEVICE_NOT_FOUND, who_am_i);
        return -1;
    }

    result = imu_dmp_start();
    if (result != 0)
    {
        imu_publish_status(IMU_STATUS_DMP_ERROR, who_am_i);
        return result;
    }

    DL_GPIO_clearInterruptStatus(MPU6050_INT_PORT,
                                 MPU6050_INT_INT_PIN_PIN);
    imu_initialized = 1U;
    imu_publish_status(IMU_STATUS_READY, who_am_i);
    return 0;
}

void imu_data_ready_isr(void)
{
    imu_interrupt_count++;
    if (imu_initialized != 0U)
    {
        imu_sample_pending = 1U;
    }
}

void imu_service(void)
{
    uint8_t packet_count = 0U;
    unsigned char more = 0U;

    if (imu_initialized == 0U || imu_sample_pending == 0U)
    {
        return;
    }

    imu_sample_pending = 0U;
    do
    {
        short gyro[3];
        short accel[3];
        short sensors = 0;
        unsigned long timestamp = 0UL;
        long quat_q30[4];
        float q0;
        float q1;
        float q2;
        float q3;
        float norm_sq;
        float inv_norm;
        float yaw_wrapped;
        float yaw_relative;
        imu_sample_t sample;
        int result;

        result = dmp_read_fifo(gyro, accel, quat_q30, &timestamp,
                               &sensors, &more);
        if (result == MPU_FIFO_NO_PACKET)
        {
            imu_sample_pending = 1U;
            return;
        }
        if (result != 0)
        {
            imu_fifo_error_count++;
            imu_publish_status(IMU_STATUS_FIFO_ERROR,
                               IMU_WHO_AM_I_VALUE);
            return;
        }

        packet_count++;
        if ((sensors & INV_WXYZ_QUAT) == 0)
        {
            continue;
        }

        q0 = (float)quat_q30[0] / IMU_Q30_SCALE;
        q1 = (float)quat_q30[1] / IMU_Q30_SCALE;
        q2 = (float)quat_q30[2] / IMU_Q30_SCALE;
        q3 = (float)quat_q30[3] / IMU_Q30_SCALE;
        norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        if (norm_sq < 0.5f || norm_sq > 1.5f)
        {
            imu_fifo_error_count++;
            continue;
        }

        inv_norm = 1.0f / sqrtf(norm_sq);
        q0 *= inv_norm;
        q1 *= inv_norm;
        q2 *= inv_norm;
        q3 *= inv_norm;

        memset(&sample, 0, sizeof(sample));
        sample.quaternion[0] = q0;
        sample.quaternion[1] = q1;
        sample.quaternion[2] = q2;
        sample.quaternion[3] = q3;
        sample.roll_deg = atan2f(q0 * q1 + q2 * q3,
                                 0.5f - q1 * q1 - q2 * q2) *
                          IMU_RAD_TO_DEG;
        sample.pitch_deg = asinf(imu_clamp_unit(
                                     2.0f * (q0 * q2 - q3 * q1))) *
                           IMU_RAD_TO_DEG;
        yaw_wrapped = atan2f(q0 * q3 + q1 * q2,
                             0.5f - q2 * q2 - q3 * q3) *
                      IMU_RAD_TO_DEG;

        if (imu_yaw_started == 0U)
        {
            imu_yaw_started = 1U;
            imu_last_wrapped_yaw = yaw_wrapped;
            imu_unwrapped_yaw = yaw_wrapped;
            imu_yaw_zero = yaw_wrapped;
        }
        else
        {
            imu_unwrapped_yaw +=
                imu_wrap_180(yaw_wrapped - imu_last_wrapped_yaw);
            imu_last_wrapped_yaw = yaw_wrapped;
        }
        if (imu_zero_yaw_requested != 0U)
        {
            imu_zero_yaw_requested = 0U;
            imu_yaw_zero = imu_unwrapped_yaw;
        }

        yaw_relative = imu_unwrapped_yaw - imu_yaw_zero;
        sample.yaw_continuous_deg = yaw_relative;
        sample.yaw_deg = imu_wrap_180(yaw_relative);
        sample.gyro_z_dps =
            (imu_gyro_sensitivity > 0.0f) ?
            ((float)gyro[2] / imu_gyro_sensitivity) : 0.0f;
        sample.accel_g[0] =
            (float)accel[0] / imu_accel_sensitivity;
        sample.accel_g[1] =
            (float)accel[1] / imu_accel_sensitivity;
        sample.accel_g[2] =
            (float)accel[2] / imu_accel_sensitivity;
        sample.status = IMU_STATUS_READY;
        sample.valid = 1U;
        sample.who_am_i = IMU_WHO_AM_I_VALUE;
        sample.timestamp_ms = (uint32_t)timestamp;
        sample.interrupt_count = imu_interrupt_count;
        sample.sample_count = ++imu_sample_count;
        sample.fifo_error_count = imu_fifo_error_count;
        imu_publish(&sample);
    }
    while (more != 0U &&
           packet_count < IMU_MAX_FIFO_PACKETS_PER_SERVICE);

    if (more != 0U)
    {
        imu_sample_pending = 1U;
    }
}

void imu_request_yaw_zero(void)
{
    imu_zero_yaw_requested = 1U;
}

void imu_get_snapshot(imu_sample_t *sample)
{
    uint32_t primask;

    if (sample == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *sample = imu_published;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

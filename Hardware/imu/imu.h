#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef enum
{
    IMU_STATUS_NOT_INITIALIZED = 0,
    IMU_STATUS_INITIALIZING,
    IMU_STATUS_READY,
    IMU_STATUS_DEVICE_NOT_FOUND,
    IMU_STATUS_DMP_ERROR,
    IMU_STATUS_FIFO_ERROR
} imu_status_t;

typedef struct
{
    imu_status_t status;
    uint8_t valid;
    uint8_t who_am_i;
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_continuous_deg;
    float gyro_z_dps;
    float accel_g[3];
    uint32_t timestamp_ms;
    uint32_t interrupt_count;
    uint32_t sample_count;
    uint32_t fifo_error_count;
} imu_sample_t;

int imu_init(void);
void imu_data_ready_isr(void);
void imu_service(void);
void imu_request_yaw_zero(void);
void imu_get_snapshot(imu_sample_t *sample);

#endif

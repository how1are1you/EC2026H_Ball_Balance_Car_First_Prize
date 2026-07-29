#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include <stdint.h>

#define ENCODER_CALIBRATION_DISTANCE_MM             (1500U)
#define ENCODER_CALIBRATION_COUNTS_PER_REV          (728U)
#define ENCODER_LEFT_COUNTS_PER_METER               (3507.66f)
#define ENCODER_RIGHT_COUNTS_PER_METER              (3521.14f)

#define ENCODER_AUTO_DISTANCE_MM                     (4000U)
#define ENCODER_AUTO_CRUISE_SPEED_MPS                (0.20f)
#define ENCODER_AUTO_ACCELERATION_MPS2               (0.20f)
#define ENCODER_AUTO_SYNC_KP                         (1.00f)
#define ENCODER_AUTO_MAX_SYNC_CORRECTION_MPS         (0.03f)
#define ENCODER_AUTO_STOP_TOLERANCE_MM               (3U)
#define ENCODER_AUTO_MAX_DISTANCE_MISMATCH_MM        (100U)
#define ENCODER_AUTO_TIMEOUT_TICKS                   (8000U)
#define ENCODER_AUTO_STILL_TICKS                     (40U)

#define ENCODER_AUTO_FAULT_NONE                      (0U)
#define ENCODER_AUTO_FAULT_TIMEOUT                   (1U)
#define ENCODER_AUTO_FAULT_MISMATCH                  (2U)

typedef enum
{
    ENCODER_CALIBRATION_IDLE = 0,
    ENCODER_CALIBRATION_RUNNING,
    ENCODER_CALIBRATION_SETTLING,
    ENCODER_CALIBRATION_DONE
} EncoderCalibrationState;

typedef enum
{
    ENCODER_CALIBRATION_MANUAL = 0,
    ENCODER_CALIBRATION_AUTO
} EncoderCalibrationMode;

extern volatile EncoderCalibrationState encoder_calibration_state;
extern volatile EncoderCalibrationMode encoder_calibration_mode;
extern volatile int32_t encoder_calibration_left_count;
extern volatile int32_t encoder_calibration_right_count;
extern volatile uint32_t encoder_calibration_left_counts_per_meter;
extern volatile uint32_t encoder_calibration_right_counts_per_meter;
extern volatile uint32_t encoder_calibration_left_distance_mm;
extern volatile uint32_t encoder_calibration_right_distance_mm;
extern volatile uint8_t encoder_calibration_auto_fault;

void encoder_calibration_reset(void);
void encoder_calibration_start(void);
void encoder_calibration_stop(void);
void encoder_calibration_start_auto(void);
void encoder_calibration_stop_auto(void);
void encoder_calibration_update(int encoder_count_a, int encoder_count_b);
uint8_t encoder_calibration_auto_control(
    float *left_target_speed,
    float *right_target_speed);
void encoder_calibration_service(void);

#endif

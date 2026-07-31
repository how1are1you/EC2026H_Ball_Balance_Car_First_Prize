#include <assert.h>
#include <math.h>
#include <stdint.h>

volatile unsigned long tick_ms;
volatile float vision_ball_position_mm;
volatile float vision_ball_velocity_mm_s;
volatile uint32_t vision_ball_frame_count;
volatile uint32_t vision_ball_last_update_ms;
volatile uint8_t vision_ball_position_valid;

void servo_set_pulse_us(uint16_t pulse_us)
{
    (void)pulse_us;
}

#include "../Control/ball_balance.c"

static void raw_ay_is_converted_to_mps2(void)
{
    ball_balance_init();
    ball_balance_set_vehicle_acceleration_from_raw_ay(0.05f);
    assert(fabsf(ball_balance_vehicle_acceleration_mps2 -
                 0.4903325f) < 0.001f);
}

static void converted_raw_ay_uses_existing_limit(void)
{
    ball_balance_init();
    ball_balance_set_vehicle_acceleration_from_raw_ay(-0.50f);
    assert(fabsf(ball_balance_vehicle_acceleration_mps2 +
                 BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2) <
           0.001f);
}

int main(void)
{
    raw_ay_is_converted_to_mps2();
    converted_raw_ay_uses_existing_limit();
    return 0;
}

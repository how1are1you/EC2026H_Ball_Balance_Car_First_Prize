#ifndef __SHOW_H
#define __SHOW_H
#include "board.h"
extern float Velocity_Left,Velocity_Right;//左轮速度、右轮速度
void oled_show(void);
void imu_startup_oled_show(uint8_t seconds_remaining);
void APP_Show(void);
void DataScope(void);
void oled_show_once(void);
#endif

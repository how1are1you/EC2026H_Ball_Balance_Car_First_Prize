/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "ball_balance.h"
#include "ball_hold_lap.h"
#include "ball_static_task.h"
#include "imu/imu.h"
#include "servo.h"

#define IMU_STARTUP_STABILIZE_MS (10000UL)

u8 Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;                                                    // 电机PWM变量 应是Motor的
u8 PID_Send;                                                                    // 延时和调参相关变量
float RC_Velocity = 200, RC_Turn_Velocity, Move_X, Move_Y, Move_Z, PS2_ON_Flag; // 遥控控制的速度
float Velocity_Left, Velocity_Right;                                            // 车轮速度(mm/s)
u16 test_num, show_cnt;
float Voltage = 0;
int Servo_Init = SERVO_NEUTRAL_PULSE_US;

static void imu_wait_for_stabilization(void)
{
    unsigned long start_ms = tick_ms;
    uint8_t displayed_seconds = 0U;

    while ((unsigned long)(tick_ms - start_ms) <
           IMU_STARTUP_STABILIZE_MS)
    {
        unsigned long elapsed_ms =
            (unsigned long)(tick_ms - start_ms);
        uint8_t remaining_seconds = (uint8_t)((IMU_STARTUP_STABILIZE_MS - elapsed_ms + 999UL) / 1000UL);

        if (remaining_seconds != displayed_seconds)
        {
            displayed_seconds = remaining_seconds;
            imu_startup_oled_show(remaining_seconds);
        }

        imu_service();
        delay_ms(1U);
    }
}

int main(void)
{
    int imu_result;

    // 系统初始化
    SYSCFG_DL_init(); // 初始化系统配置
    vision_uart_reset();
    control_uart_reset();
    servo_init();
    ball_balance_init();
    ball_hold_lap_init();
    ball_static_task_init();
    // 清除所有外设的中断挂起状态
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN); // 编码器A与MPU6050中断
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);            // 编码器B中断
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);         // UART0串口中断
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);         // UART1串口中断
    // 使能各外设的中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN); // 开启编码器A与MPU6050中断
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);            // 开启编码器B中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);         // 开启UART0中断
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);         // 开启UART1中断
    // 定时器和ADC相关中断配置
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN); // 清除定时器0中断挂起
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);       // 开启定时器0中断
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
    OLED_Init(); // 初始化OLED显示屏
    imu_result = imu_init();
    if (imu_result == 0)
    {
        imu_wait_for_stabilization();
    }
    // 主循环
    while (1)
    {
        imu_service();
        control_uart_service();
        ball_static_task_service();
        Voltage = Get_battery_volt(); // 采样小车当前电压
        oled_show();                  //  OLED显示更新
    }
}

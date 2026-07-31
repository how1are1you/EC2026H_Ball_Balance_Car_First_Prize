# 原始 Ay 小球前馈设计

## 目标

在 `BALL LAP` 和 `BALL HOLD` 运行期间，以 MPU6050 原始 `Y` 轴加速度作为小车纵向加速度，驱动现有小球平衡的加速度前馈。用户已通过手动倾斜确认 `Y` 为小车纵向轴。

## 输入与符号

直接使用最新有效 IMU 快照的原始值：

```c
vehicle_acceleration_mps2 = sample.accel_g[1] * 9.80665f;
```

前进方向定义为正 Y；前进加速对应正值，前进减速或后退加速对应负值。此设计刻意不减去车身倾斜造成的重力分量，不做姿态补偿、低通滤波，也不与规划起步加速度叠加。

## 接入位置与行为

仅修改 `Control/control.c` 的 5 ms `TIMER_0_INST_IRQHandler()`：它已经负责启用小球控制、调用 `ball_balance_set_vehicle_acceleration()` 和随后调用 `ball_balance_update()`，并已包含 `imu/imu.h`。

在小球控制启用的 `BALL LAP` 或 `BALL HOLD` 运行状态中读取一次 `imu_sample_t` 快照：

- 当 IMU 状态为 `IMU_STATUS_READY` 且样本有效时，传入上述原始 Ay 换算值；
- 当 IMU 不可用、车辆停止、菜单打开或不处于这两个运行模式时，传入 `0.0f`；
- 继续使用 `ball_balance_set_vehicle_acceleration()` 的现有 `+/-1.0 m/s2` 限幅；
- 保持 `ball_balance` 的既有前馈增益、脉宽限幅和 `BALL STATIC` 行为不变。

这将替换现有仅在直道起步阶段提供的 `StraightTurnStartupAccelerationMps2` 输入。`straight_turn_test` 仍负责电机速度规划，但不再是小球前馈的加速度来源。

## 故障与安全

若 IMU 暂时无有效样本，前馈归零；位置/速度 PID 继续由现有逻辑控制。不会因该前馈输入本身增加新的故障状态或改变车辆循迹停止策略。

## 验证

1. Keil 目标 `MSPM0G3507_Project` 全量构建成功且无新警告。
2. 进入 `BALL LAP` 或 `BALL HOLD`，以低速运行并观察小球在车辆加速、减速和匀速阶段的偏移趋势。
3. 若加速时小球偏移加剧，先将 `BALL_BALANCE_ACCELERATION_FF_DIRECTION` 改为相反符号后复测；再微调 `BALL_BALANCE_ACCELERATION_FF_US_PER_MPS2`，每次只改变一个参数。
4. 确认退出运行模式、IMU 不可用或车辆停止时，舵机前馈不会残留。

## 范围外

本次不修改 `empty.syscfg`、IMU 驱动、生成的 TI 配置、串口协议、OLED 页面、视觉定位、规划加速度计算或现有 PID 参数。

# 原始 Ay 小球前馈 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `BALL LAP` 和 `BALL HOLD` 运行期间，以最新有效 IMU 原始 `Ay` 换算的纵向加速度驱动现有小球加速度前馈。

**Architecture:** `Control/control.c` 保持 IMU 读取职责：只在现有小球运行条件成立时读取快照，并把 `sample.accel_g[1]` 交给 `ball_balance`。`Control/ball_balance.c` 新增一个小型转换接口，将原始 g 值乘以标准重力加速度后复用已有 `ball_balance_set_vehicle_acceleration()`，从而保留既有 `+/-1.0 m/s2` 限幅并让转换逻辑可由宿主测试覆盖。

**Tech Stack:** C99、GCC 宿主测试、TI MSPM0 DriverLib、MPU6050 DMP、Keil uVision/ARMCLANG。

## Global Constraints

- 原始加速度输入固定为 `sample.accel_g[1]`；不做重力补偿、姿态补偿、低通滤波或与 `StraightTurnStartupAccelerationMps2` 融合。
- 仅在菜单关闭、`BALL LAP` 或 `BALL HOLD` 模式、且 `Flag_Stop == 0U` 时使用有效 IMU 样本；其他条件下前馈输入为 `0.0f`。
- 仅接受 `IMU_STATUS_READY` 且 `sample.valid != 0U` 的 IMU 快照；无效快照使前馈归零，不新增故障状态。
- 继续使用既有 `BALL_BALANCE_VEHICLE_ACCELERATION_LIMIT_MPS2`、前馈增益和舵机限幅。
- 不修改 `empty.syscfg`、`Hardware/imu/`、生成的 TI 配置、视觉 UART、OLED、PID 参数或 Keil 项目文件。
- 不提交用户已有的 `Control/ball_balance.h`、`empty.syscfg`、`keil/Objects/` 或 `.uvguix.*` 改动；若本任务需要修改 `Control/ball_balance.h`，暂存时只加入本任务新增的接口行，并先检查差异。

---

## File Structure

- Create: `tests/ball_balance_raw_ay_test.c` — 直接包含 `ball_balance.c`，验证原始 Ay 的 g→m/s² 换算及沿用的限幅。
- Modify: `Control/ball_balance.h` — 声明原始 Ay 换算接口。
- Modify: `Control/ball_balance.c` — 实现换算接口，复用已有加速度限幅函数。
- Modify: `Control/control.c` — 读取有效 IMU 快照，并在现有小球运行门控下调用换算接口。

### Task 1: 为原始 Ay 换算建立失败测试

**Files:**
- Create: `tests/ball_balance_raw_ay_test.c`
- Test: `tests/ball_balance_raw_ay_test.c`

**Interfaces:**
- Consumes: `void ball_balance_init(void)`、`volatile float ball_balance_vehicle_acceleration_mps2`。
- Produces: `void ball_balance_set_vehicle_acceleration_from_raw_ay(float raw_ay_g)`，参数单位为 g。

- [ ] **Step 1: 写入失败测试**

创建 `tests/ball_balance_raw_ay_test.c`，提供 `ball_balance.c` 所需的视觉与舵机测试桩，并包含生产源文件：

```c
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
```

- [ ] **Step 2: 运行测试并确认因接口不存在而失败**

Run:

```powershell
& 'D:\mingw\mingw64\bin\gcc.exe' -std=c99 -IControl -IHardware tests\ball_balance_raw_ay_test.c -lm -o tmp\ball_balance_raw_ay_test.exe
```

Expected: 编译失败，错误指出 `ball_balance_set_vehicle_acceleration_from_raw_ay` 未声明或未定义。

### Task 2: 实现原始 Ay 换算接口并验证测试

**Files:**
- Modify: `Control/ball_balance.h:61`
- Modify: `Control/ball_balance.c:191-197`
- Test: `tests/ball_balance_raw_ay_test.c`

**Interfaces:**
- Consumes: `float raw_ay_g`，单位 g。
- Produces: 更新的 `ball_balance_vehicle_acceleration_mps2`，单位 m/s² 且受既有限幅保护。

- [ ] **Step 1: 在头文件声明接口**

在现有 `ball_balance_set_vehicle_acceleration()` 声明之后添加：

```c
void ball_balance_set_vehicle_acceleration_from_raw_ay(
    float raw_ay_g);
```

- [ ] **Step 2: 写入最小生产实现**

在 `Control/ball_balance.c` 中已有 `ball_balance_set_vehicle_acceleration()` 之后添加：

```c
void ball_balance_set_vehicle_acceleration_from_raw_ay(
    float raw_ay_g)
{
    ball_balance_set_vehicle_acceleration(
        raw_ay_g * 9.80665f);
}
```

不要在这里增加滤波、姿态补偿或额外限幅。

- [ ] **Step 3: 运行测试并确认通过**

Run:

```powershell
& 'D:\mingw\mingw64\bin\gcc.exe' -std=c99 -IControl -IHardware tests\ball_balance_raw_ay_test.c -lm -o tmp\ball_balance_raw_ay_test.exe
& .\tmp\ball_balance_raw_ay_test.exe
```

Expected: 两条断言均通过，进程退出码为 0。

### Task 3: 在现有小球运行门控下接入 IMU Ay

**Files:**
- Modify: `Control/control.c:70-150`
- Test: `tests/ball_balance_raw_ay_test.c`

**Interfaces:**
- Consumes: `imu_get_snapshot(&sample)`、`sample.status`、`sample.valid`、`sample.accel_g[1]`，以及现有运行模式和 `Flag_Stop`。
- Produces: 对 `ball_balance_set_vehicle_acceleration_from_raw_ay()` 或 `ball_balance_set_vehicle_acceleration(0.0f)` 的单次调用，发生在每个 5 ms 控制周期且早于 `ball_balance_update()`。

- [ ] **Step 1: 在定时器中断局部变量区声明 IMU 快照**

紧接 `static int lastRunMode = -1;` 之后声明：

```c
    imu_sample_t sample;
```

- [ ] **Step 2: 替换规划加速度前馈输入**

用下列代码替换现有 `ball_balance_set_vehicle_acceleration(...)` 调用：

```c
            if (Menu_Active == 0U &&
                (Run_Mode == RUN_MODE_BALL_LAP ||
                 Run_Mode == RUN_MODE_BALL_HOLD_LAP) &&
                Flag_Stop == 0U)
            {
                imu_get_snapshot(&sample);
                if (sample.status == IMU_STATUS_READY &&
                    sample.valid != 0U)
                {
                    ball_balance_set_vehicle_acceleration_from_raw_ay(
                        sample.accel_g[1]);
                }
                else
                {
                    ball_balance_set_vehicle_acceleration(0.0f);
                }
            }
            else
            {
                ball_balance_set_vehicle_acceleration(0.0f);
            }
```

该代码不引用 `StraightTurnStartupAccelerationMps2`，并保留后续 `ball_balance_update()` 的位置不变。

- [ ] **Step 3: 重新运行宿主测试**

Run:

```powershell
& 'D:\mingw\mingw64\bin\gcc.exe' -std=c99 -IControl -IHardware tests\ball_balance_raw_ay_test.c -lm -o tmp\ball_balance_raw_ay_test.exe
& .\tmp\ball_balance_raw_ay_test.exe
```

Expected: 两条断言通过，退出码为 0。

- [ ] **Step 4: 全量构建 Keil 固件**

Run:

```powershell
& 'D:\Infineon\Keli\Keil_v5\UV4\UV4.exe' -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -t MSPM0G3507_Project
```

Expected: 进程退出码为 0，`keil/MSPM0G3507_Project_build.log` 包含 `0 Error(s), 0 Warning(s)`。

- [ ] **Step 5: 目标板验证与调参**

1. 烧录固件，进入 `BALL LAP` 或 `BALL HOLD`。
2. 先以低速运行，分别观察起步、匀速、减速时小球位置。
3. 若加速时小球偏移增大，将 `BALL_BALANCE_ACCELERATION_FF_DIRECTION` 的符号从 `1.0f` 改为 `-1.0f` 后单独复测。
4. 在正确方向下，每次仅调整 `BALL_BALANCE_ACCELERATION_FF_US_PER_MPS2`，记录小球最大误差并避免一次同时调整 PID。
5. 停车、退出模式或断开 IMU 时确认舵机不会保持前馈偏置。

- [ ] **Step 6: 只提交本任务文件**

Run:

```powershell
rtk git add -- Control/control.c Control/ball_balance.c tests/ball_balance_raw_ay_test.c
@'
diff --git a/Control/ball_balance.h b/Control/ball_balance.h
--- a/Control/ball_balance.h
+++ b/Control/ball_balance.h
@@ -59,6 +59,8 @@ extern volatile ball_balance_status_t ball_balance_status;
 void ball_balance_init(void);
 void ball_balance_set_enabled(uint8_t enabled);
 void ball_balance_set_vehicle_acceleration(float acceleration_mps2);
+void ball_balance_set_vehicle_acceleration_from_raw_ay(
+    float raw_ay_g);
 void ball_balance_set_reference(
     float position_mm,
     float velocity_mm_s);
'@ | rtk git apply --cached
rtk git diff --cached --check
rtk git commit -m "ball: feed forward raw longitudinal acceleration"
```

Expected: 提交只包含本任务的 `Control/control.c`、`Control/ball_balance.c`、`Control/ball_balance.h` 和 `tests/ball_balance_raw_ay_test.c`，不包含用户现有的 `empty.syscfg`、Keil 产物或 `.uvguix.*`。

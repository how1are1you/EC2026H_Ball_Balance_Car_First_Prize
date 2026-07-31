# IMU 加速度 OLED 调试显示 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 IMU DEBUG 模式的 OLED 上实时显示 MPU6050 三轴原始加速度，供手推小车确认前进轴及正负方向。

**Architecture:** 保持 `Hardware/imu/imu.c` 的 200 Hz DMP 采样和 `imu_sample_t.accel_g[3]` 发布接口不变。仅扩展 `Control/show.c` 中已有的 `imu_debug_oled_show()`：它获取一个一致的 IMU 快照，并与已有状态、航向角和 Z 轴角速度一起绘制三轴加速度。

**Tech Stack:** C99、TI MSPM0 DriverLib、MPU6050 DMP、128x64 OLED、Keil uVision/ARMCLANG。

## Global Constraints

- 不修改 `Hardware/imu/imu.c`、`Hardware/imu/imu.h`、`empty.syscfg` 或生成的 `ti_msp_dl_config.c/.h`。
- 不新增串口输出、基线标定、峰值保持、自动方向判断或重力补偿。
- 不改变 `BALL LAP`、`BALL HOLD`、`STATIC HYB`、视觉 UART1 或小球平衡前馈行为。
- `IMU DEBUG` 保持现有停机逻辑；OLED 刷新仍由 `oled_show()` 的 50 ms 限速控制。
- 不提交 `keil/Objects/`、`.uvguix.*` 或其他本地 Keil 产物。

---

## File Structure

- Modify: `Control/show.c` — 复用现有 IMU 快照与有符号小数绘制函数，在 IMU DEBUG 页面占用 y=30、40、50 三行显示 `AX`、`AY`、`AZ`，单位 `g`。
- Modify: `docs/superpowers/specs/2026-07-31-imu-accel-display-design.md` — 仅在实现与设计出现不一致时更新；预期无需改动。
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` — 使用现有 `MSPM0G3507_Project` 目标编译，不编辑项目文件。

硬件 OLED 绘制没有宿主机替身或自动化测试框架；本变更不引入独立算法，因此以 Keil 编译和目标板手推观察作为验证，而不为单行绘制创建额外模块或测试桩。

### Task 1: 扩展 IMU DEBUG OLED 页面

**Files:**
- Modify: `Control/show.c:217-260`
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: `void imu_get_snapshot(imu_sample_t *sample)` 和 `imu_sample_t.accel_g[3]`，其单位为 `g`。
- Reuses: `static void imu_show_signed_tenths(uint8_t x, uint8_t y, float value)`。
- Produces: `IMU DEBUG` 页面在 50 ms 刷新周期内显示 `AX`、`AY`、`AZ`。

- [ ] **Step 1: 确认当前 IMU DEBUG 页面仅显示状态、航向角和 GZ**

Run: `rtk rg -n -A 50 -B 5 "static void imu_debug_oled_show" Control/show.c`

Expected: 函数获取 `imu_sample_t sample`，在 y=0、10、20 行显示状态、`Y:` 和 `GZ:`，且尚未引用 `sample.accel_g`。

- [ ] **Step 2: 以最小改动加入三轴绘制**

在 `imu_debug_oled_show()` 的 `GZ` 绘制之后、`OLED_Refresh_Gram()` 之前加入以下代码；不要改变状态、航向角或 GZ 的既有绘制：

```c
    oled_show_text(0, 30, "AX:");
    imu_show_signed_tenths(24, 30, sample.accel_g[0]);
    oled_show_text(66, 30, "g");

    oled_show_text(0, 40, "AY:");
    imu_show_signed_tenths(24, 40, sample.accel_g[1]);
    oled_show_text(66, 40, "g");

    oled_show_text(0, 50, "AZ:");
    imu_show_signed_tenths(24, 50, sample.accel_g[2]);
    oled_show_text(66, 50, "g");
```

`imu_show_signed_tenths()` 会显示明确的 `+` 或 `-`，并保留一位小数；手推时通过变化最大的轴及前后推动时的符号反转确定方向。

- [ ] **Step 3: 静态检查受影响源文件**

Run: `rtk git diff --check -- Control/show.c; rtk git diff -- Control/show.c`

Expected: 无空白错误；只有 `imu_debug_oled_show()` 新增三轴显示，未出现 UART、IMU 驱动、SysConfig 或小球控制改动。

- [ ] **Step 4: 全量构建 Keil 固件**

Run: `rtk powershell.exe -NoProfile -Command "& 'D:\Infineon\Keli\Keil_v5\UV4\UV4.exe' -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -t MSPM0G3507_Project"`

Expected: 进程退出码为 0，构建日志含 `0 Error(s)`，且没有由 `Control/show.c` 引入的新警告。

- [ ] **Step 5: 在目标板上验证方向观察**

1. 烧录本构建生成的固件，进入菜单中的 `IMU DEBUG`。
2. 确认 OLED 同时显示 `AX`、`AY`、`AZ`，单位 `g`，并且车辆保持停机。
3. 保持小车静止，记录接近 +/-1.0 g 的重力轴。
4. 沿车辆前进方向快速手推一次，再沿反方向手推一次。
5. 记录变化最明显的加速度轴；确认两次推动的该轴符号相反。此轴和符号将作为下一阶段实际加速度前馈设计的输入依据。

- [ ] **Step 6: 提交源代码改动（不含现有用户改动及 Keil 产物）**

Run:

```powershell
rtk git add -- Control/show.c
rtk git diff --cached --check
rtk git commit -m "imu: show accelerometer axes in debug mode"
```

Expected: 提交仅包含 `Control/show.c`；不要暂存当前工作区中已存在的 `Control/ball_balance.h`、`keil/Objects/` 或 `.uvguix.*` 修改。

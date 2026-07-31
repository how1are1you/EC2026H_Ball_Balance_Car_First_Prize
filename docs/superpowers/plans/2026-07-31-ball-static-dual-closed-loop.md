# 静态滚球双端闭环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使静态滚球任务以位置闭环控制钢球到达 +5 cm、短暂停稳后折返，并在 -5 cm 稳定后完成。

**Architecture:** 保留现有 `ball_static_task` 单一状态机和菜单接口，将 `MOVE_POS`、`HOLD_POS`、`MOVE_NEG`、`HOLD_NEG` 分别改为正向 PID、正向稳定确认、负向 PID、负向稳定确认。完成态继续提供 -5 cm 位置参考；状态机独立统计正端峰值、稳定计时和总任务超时。

**Tech Stack:** C99、MSPM0 DriverLib、现有 `ball_balance` PID、Keil uVision/ARMCLANG、主机端 C `assert` 合同测试。

## Global Constraints

- 位置统一使用 `vision_ball_position_mm`（mm）；经标定的物理目标为 +50 mm、-50 mm。
- 正、负端进入容差为 ±5 mm，稳定速度阈值为 10 mm/s。
- 正端连续稳定 100 ms 后折返；负端连续稳定 300 ms 后完成。
- 从启动至完成或故障的总时长不得超过 5000 ms。
- 正向端点最大误差为正向运动期间的峰值相对 +50 mm 的绝对误差，目标不超过 10 mm。
- `DONE` 状态继续保持 -5 cm PID 参考；视觉失效和超时进入 `FAULT` 并使舵机回中。

---

## File Structure

- `Control/ball_static_task.c`：双端闭环状态机、稳定计时、端点峰值统计和超时保护。
- `Control/ball_static_task.h`：保持现有任务 API 与状态枚举；不为内部计时器新增对外 API。
- `tests/ball_static_task_test.c`：在主机上以视觉/舵机/PID 存根驱动状态机，验证状态切换与故障边界。
- `Control/show.c`：仅当状态文字不能准确说明正负端 PID/稳定阶段时，更新显示文本；否则不修改。

### Task 1: 为双端闭环状态机建立失败测试

**Files:**
- Create: `tests/ball_static_task_test.c`
- Modify: none
- Test: `tests/ball_static_task_test.c`

**Interfaces:**
- Consumes: `ball_static_task_init(void)`, `ball_static_task_start(void)`, `ball_static_task_update(void)`, `ball_static_task_controller_enabled(void)`。
- Produces: 一个可由主机 C 编译器运行的测试二进制；该文件为 `tick_ms`、视觉位置/速度、`ball_balance_set_reference`、`servo_set_pulse_us` 和 `servo_get_pulse_us` 提供存根。

- [ ] **Step 1: 写入失败测试，覆盖正端确认、负端完成和超时**

```c
static void publish_sample(float position_mm, float velocity_mm_s,
                           uint32_t sample_ms)
{
    tick_ms = sample_ms;
    vision_ball_position_mm = position_mm;
    vision_ball_velocity_mm_s = velocity_mm_s;
    vision_ball_last_update_ms = sample_ms;
    vision_ball_position_valid = 1U;
}

static void ready_and_start(void)
{
    ball_static_task_init();
    publish_sample(0.0f, 0.0f, 1U);
    ball_static_task_update();
    publish_sample(0.0f, 0.0f, 201U);
    ball_static_task_update();
    assert(ball_static_task_start() == 1U);
}

static void positive_settle_then_reverse(void)
{
    ready_and_start();
    assert(ball_static_task_controller_enabled() == 1U);
    publish_sample(48.0f, 6.0f, 500U);
    ball_static_task_update();
    publish_sample(50.0f, 4.0f, 550U);
    ball_static_task_update();
    publish_sample(50.0f, 3.0f, 650U);
    ball_static_task_update();
    assert(fabsf(ball_static_target_mm + 50.0f) < 0.01f);
    assert(fabsf(ball_static_positive_max_error_mm) < 0.01f);
}

static void negative_settle_completes_while_pid_stays_enabled(void)
{
    positive_settle_then_reverse();
    publish_sample(-50.0f, 4.0f, 900U);
    ball_static_task_update();
    publish_sample(-50.0f, 3.0f, 1200U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_DONE);
    assert(ball_static_task_controller_enabled() == 1U);
    assert(fabsf(ball_balance_target_mm + 50.0f) < 0.01f);
}

static void task_faults_after_five_seconds(void)
{
    ready_and_start();
    publish_sample(20.0f, 20.0f, 5202U);
    ball_static_task_update();
    assert(ball_static_state == BALL_STATIC_FAULT);
    assert(ball_static_fault == BALL_STATIC_FAULT_TOTAL_TIMEOUT);
}
```

- [ ] **Step 2: 编译并确认测试在旧实现上失败**

Run: `clang -std=c99 -Wall -Wextra -IControl -IHardware tests/ball_static_task_test.c -lm -o tests/ball_static_task_test.exe`

Expected: 编译成功；运行 `tests/ball_static_task_test.exe` 后因正向阶段尚未启用 PID 或未在正端稳定后切换到负目标而断言失败。

- [ ] **Step 3: 保留测试存根的硬件边界**

测试文件应仅存根下列硬件/外部符号，不链接 MCU 驱动：`tick_ms`、`vision_ball_position_mm`、`vision_ball_velocity_mm_s`、`vision_ball_last_update_ms`、`vision_ball_position_valid`、`ball_balance_set_reference`、`servo_set_pulse_us`、`servo_get_pulse_us`。其余状态数据使用 `ball_static_task.c` 的真实定义。

- [ ] **Step 4: 再次编译失败测试，确保其能表达目标行为**

Run: `clang -std=c99 -Wall -Wextra -IControl -IHardware tests/ball_static_task_test.c -lm -o tests/ball_static_task_test.exe`

Expected: 编译成功，运行仍失败，且失败来自新的正端/负端/超时断言而不是未定义符号。

- [ ] **Step 5: 提交测试基线**

```powershell
git add tests/ball_static_task_test.c
git commit -m "test: cover static ball dual-loop task"
```

### Task 2: 实现双端 PID、稳定确认和完成保持

**Files:**
- Modify: `Control/ball_static_task.c:9-24, 53-56, 176-215, 253-379`
- Modify: `Control/ball_static_task.h:6-16`（仅在需要将 `HOLD_POS`/`HOLD_NEG` 重命名为更清晰状态时）
- Modify: `Control/show.c:696-724`（仅在枚举名称变化时）
- Test: `tests/ball_static_task_test.c`

**Interfaces:**
- Consumes: `ball_balance_set_reference(float position_mm, float velocity_mm_s)`；`vision_ball_position_mm` 和 `vision_ball_velocity_mm_s`；`tick_ms`。
- Produces: `ball_static_task_update(void)` 在运动和稳定阶段持续更新 PID 目标；`ball_static_task_controller_enabled(void)` 对所有正、负 PID 阶段及 `DONE` 返回 `1U`。

- [ ] **Step 1: 添加具名的闭环阈值和内部稳定计时器**

在 `ball_static_task.c` 的现有宏旁增加以下精确常量，并删除不再参与控制切换的开环阈值宏：

```c
#define BALL_STATIC_POS_TARGET_MM (50.0f)
#define BALL_STATIC_NEG_TARGET_MM (-50.0f)
#define BALL_STATIC_SETTLE_TOLERANCE_MM (5.0f)
#define BALL_STATIC_SETTLE_VELOCITY_MM_S (10.0f)
#define BALL_STATIC_POS_SETTLE_TIME_MS (100UL)
#define BALL_STATIC_NEG_SETTLE_TIME_MS (300UL)
#define BALL_STATIC_TOTAL_TIMEOUT_MS (5000UL)

static uint32_t settle_start_ms;
```

如视觉坐标标定显示物理 -5 cm 对应非 -50 mm 的固定坐标，可只修改 `BALL_STATIC_NEG_TARGET_MM`；不得通过放宽完成容差补偿标定误差。

- [ ] **Step 2: 使整个任务运动阶段使用 PID 参考值**

在 `ball_static_task_start` 中初始化 `positive_peak_mm`、`settle_start_ms`，进入 `BALL_STATIC_MOVE_POS` 后立即调用：

```c
ball_balance_set_reference(BALL_STATIC_POS_TARGET_MM, 0.0f);
```

将 `ball_static_task_controller_enabled` 改为对 `BALL_STATIC_MOVE_POS`、`BALL_STATIC_HOLD_POS`、`BALL_STATIC_MOVE_NEG`、`BALL_STATIC_HOLD_NEG` 和 `BALL_STATIC_DONE` 返回 `1U`。这些状态不再调用 `apply_pulse` 施加固定开环脉宽。

- [ ] **Step 3: 实现正端稳定确认和折返切换**

在 `BALL_STATIC_MOVE_POS` 和 `BALL_STATIC_HOLD_POS` 中每次更新正向峰值，始终保持正向 PID 参考：

```c
if (position_mm > positive_peak_mm)
{
    positive_peak_mm = position_mm;
}
ball_balance_set_reference(BALL_STATIC_POS_TARGET_MM, 0.0f);
```

当 `abs(position_mm - BALL_STATIC_POS_TARGET_MM) <= BALL_STATIC_SETTLE_TOLERANCE_MM` 且 `abs(velocity_mm_s) <= BALL_STATIC_SETTLE_VELOCITY_MM_S` 时进入或继续 `BALL_STATIC_HOLD_POS`；否则清零 `settle_start_ms` 并回到 `BALL_STATIC_MOVE_POS`。确认连续 100 ms 后：

```c
ball_static_positive_max_error_mm = absolute_float(
    positive_peak_mm - BALL_STATIC_POS_TARGET_MM);
settle_start_ms = 0U;
ball_static_target_mm = BALL_STATIC_NEG_TARGET_MM;
ball_static_state = BALL_STATIC_MOVE_NEG;
ball_balance_set_reference(BALL_STATIC_NEG_TARGET_MM, 0.0f);
```

- [ ] **Step 4: 实现负端稳定完成与保持**

`BALL_STATIC_MOVE_NEG` 和 `BALL_STATIC_HOLD_NEG` 始终调用 `ball_balance_set_reference(BALL_STATIC_NEG_TARGET_MM, 0.0f)`。负端进入/持续条件为：

```c
absolute_float(position_mm - BALL_STATIC_NEG_TARGET_MM) <=
    BALL_STATIC_SETTLE_TOLERANCE_MM &&
absolute_float(velocity_mm_s) <= BALL_STATIC_SETTLE_VELOCITY_MM_S
```

连续满足 300 ms 后设 `ball_static_state = BALL_STATIC_DONE`。`BALL_STATIC_DONE` 每次更新继续写入 -5 cm PID 参考。移除以 `position_mm <= BALL_STATIC_NEG_REACHED_MM` 作为完成或误差记录条件的逻辑。

- [ ] **Step 5: 添加总超时和视觉故障保护**

在 `ball_static_elapsed_ms = now_ms - task_start_ms;` 后、状态 `switch` 前加入：

```c
if (ball_static_elapsed_ms > BALL_STATIC_TOTAL_TIMEOUT_MS)
{
    set_fault(BALL_STATIC_FAULT_TOTAL_TIMEOUT, now_ms);
    return;
}
```

除 `DONE` 外的全部运行状态都必须检查 `vision_is_fresh(now_ms)`；视觉超时进入 `BALL_STATIC_FAULT_VISION`。`DONE` 保持现有安全策略：持续提供最后的位置参考，不因一次图像漏帧把已完成的任务改判为故障。

- [ ] **Step 6: 运行主机测试并确认通过**

Run: `clang -std=c99 -Wall -Wextra -IControl -IHardware tests/ball_static_task_test.c -lm -o tests/ball_static_task_test.exe; .\tests\ball_static_task_test.exe`

Expected: 退出码 0；正端仅在稳定 100 ms 后折返，负端仅在稳定 300 ms 后进入 `DONE`，超过 5000 ms 进入超时故障。

- [ ] **Step 7: 提交实现**

```powershell
git add Control/ball_static_task.c Control/ball_static_task.h Control/show.c tests/ball_static_task_test.c
git commit -m "ball: add dual-loop static position task"
```

### Task 3: 编译固件并完成硬件验收记录

**Files:**
- Modify: none（除非编译报告新警告，才做最小修复）
- Test: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: Task 2 已通过的主机状态机测试和 MSPM0G3507 Keil 工程。
- Produces: 无新增警告的 `.axf` 固件，以及三轮可复核的实物测试数据。

- [ ] **Step 1: 执行 Keil 构建**

Run: `& 'D:\Infineon\Keli\Keil_v5\UV4\UV4.exe' -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -t MSPM0G3507_Project`

Expected: 构建成功，日志中没有新增 warning 或 error。

- [ ] **Step 2: 进行三轮静态实物测试**

每轮从视觉坐标中心启动，记录 OLED/串口的总时长、`ball_static_positive_max_error_mm`、完成时的负端位置、负端维持 300 ms 后的位置波动。合格判据为总时长不超过 5000 ms、正端最大误差不超过 10 mm、负端完成后仍保持 PID 控制。

- [ ] **Step 3: 若总时长超过 5 秒，按固定顺序调参**

先把 `BALL_STATIC_POS_SETTLE_TIME_MS` 从 100 ms 下调至 50 ms；若仍超时，再调整 `ball_balance` 的位置/速度 PID 增益。不得删除 -5 cm 的位置、速度和持续时间三重完成判据。

- [ ] **Step 4: 提交仅在验证中必要的最小参数修正**

```powershell
git add Control/ball_static_task.c Control/ball_balance.c Control/ball_balance.h
git commit -m "ball: tune static dual-loop response"
```

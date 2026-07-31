# BALL HOLD LAP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增启动时稳定采样并锁定钢球当前位置的 `BALL HOLD LAP` 模式，在通过 A 点时锁存一圈成绩，随后循迹 1.00 m 并平滑停车。

**Architecture:** 用独立的 `ball_hold_lap` 任务状态机协调视觉目标捕获、现有 `ball_balance` 闭环和扩展后的 `straight_turn_test`。将稳定窗口算法拆成无硬件依赖的 `ball_position_capture` 纯 C 模块，以便在主机上单元测试；菜单改为数据表驱动分页，原有运行模式通过兼容接口保持行为。

**Tech Stack:** C99、TI MSPM0 DriverLib、ARMCLANG/Keil uVision、MinGW GCC 主机测试、128×64 OLED。

## Global Constraints

- 舵机水平中位统一为 `SERVO_NEUTRAL_PULSE_US (1250U)`，禁止在控制代码中写死 `1250`。
- 捕获窗口至少 300 ms、至少 6 个新视觉帧、峰峰值不超过 5 mm。
- 锁存目标范围为 `-110 mm` 至 `+110 mm`。
- 有效视觉帧最大绝对位置误差验收阈值为 10 mm。
- 视觉数据超过 200 ms 未更新时车辆不停，舵机回中；视觉恢复后自动闭环。
- 完成第二个半圆视为通过 A 点并锁存一圈时间；附加 1.00 m 和制动时间不计入一圈成绩。
- 一圈时间超过 30.000 s 只记录 `OVERTIME`，不提前中止。
- 红外丢线和 IMU 失效继续沿用现有安全停车。
- 不修改 `empty.syscfg`、生成配置文件、`source/` 或 TI DriverLib。
- 不提交 `keil/Objects/`、uVision 用户设置或用户已经暂存的既有改动。

---

## File Map

- Create `Control/ball_position_capture.h`: 稳定窗口常量、状态结构和纯算法接口。
- Create `Control/ball_position_capture.c`: 新帧去重、窗口统计、稳定判定和目标均值。
- Create `tests/ball_position_capture_test.c`: 主机侧捕获算法单元测试。
- Create `Control/ball_hold_lap.h`: 新模式状态、OLED 快照变量和任务接口。
- Create `Control/ball_hold_lap.c`: 捕获、运行、成绩及故障协调。
- Modify `Control/straight_turn_test.h`: 后续循迹/制动状态、成绩变量和扩展启动接口。
- Modify `Control/straight_turn_test.c`: A 点锁时、附加 1 m、平滑制动。
- Modify `Control/control.h`: 新运行模式、菜单表接口。
- Modify `Control/control.c`: 定时调度、闭环使能、按键和分页菜单选择。
- Modify `Control/show.c`: 分页菜单和 `BALL HOLD` 状态页面。
- Modify `empty.c`: 初始化新任务。
- Modify `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`: 加入两个新增控制源文件。

---

### Task 1: 可测试的稳定位置捕获器

**Files:**
- Create: `Control/ball_position_capture.h`
- Create: `Control/ball_position_capture.c`
- Create: `tests/ball_position_capture_test.c`

**Interfaces:**
- Consumes: 新视觉帧的 `position_mm`、`sample_ms` 和单调递增 `frame_count`。
- Produces:
  - `void ball_position_capture_reset(ball_position_capture_t *capture);`
  - `ball_position_capture_result_t ball_position_capture_push(ball_position_capture_t *capture, float position_mm, uint32_t sample_ms, uint32_t frame_count, float *target_mm);`
  - 可供 OLED 读取的结构字段 `sample_count`、`elapsed_ms`、`mean_mm`、`span_mm`。

- [ ] **Step 1: 写捕获器失败测试**

```c
#include "ball_position_capture.h"
#include <assert.h>
#include <math.h>

static void test_stable_window_locks_mean(void)
{
    ball_position_capture_t capture;
    float target = 0.0f;
    const float samples[6] = {50.0f, 51.0f, 49.0f, 50.0f, 50.5f, 49.5f};
    unsigned int index;

    ball_position_capture_reset(&capture);
    for (index = 0U; index < 6U; index++)
    {
        ball_position_capture_result_t result =
            ball_position_capture_push(
                &capture, samples[index], index * 60U, index + 1U, &target);
        assert(result == ((index == 5U) ?
            BALL_POSITION_CAPTURE_LOCKED : BALL_POSITION_CAPTURE_WAITING));
    }
    assert(fabsf(target - 50.0f) < 0.01f);
}

static void test_duplicate_frame_is_ignored(void)
{
    ball_position_capture_t capture;
    float target = 0.0f;

    ball_position_capture_reset(&capture);
    (void)ball_position_capture_push(&capture, 10.0f, 0U, 1U, &target);
    (void)ball_position_capture_push(&capture, 100.0f, 100U, 1U, &target);
    assert(capture.sample_count == 1U);
    assert(capture.span_mm == 0.0f);
}

static void test_unstable_or_out_of_range_window_restarts(void)
{
    ball_position_capture_t capture;
    float target = 0.0f;

    ball_position_capture_reset(&capture);
    (void)ball_position_capture_push(&capture, 0.0f, 0U, 1U, &target);
    (void)ball_position_capture_push(&capture, 8.0f, 60U, 2U, &target);
    assert(capture.sample_count == 1U);
    assert(fabsf(capture.mean_mm - 8.0f) < 0.01f);
    (void)ball_position_capture_push(&capture, 111.0f, 120U, 3U, &target);
    assert(capture.sample_count == 0U);
}
```

- [ ] **Step 2: 运行测试并确认因接口不存在而失败**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl tests/ball_position_capture_test.c Control/ball_position_capture.c -lm -o tests/ball_position_capture_test.exe
```

Expected: FAIL，提示 `ball_position_capture.h` 或接口不存在。

- [ ] **Step 3: 创建捕获器头文件和最小实现**

```c
#define BALL_POSITION_CAPTURE_DURATION_MS (300UL)
#define BALL_POSITION_CAPTURE_MIN_FRAMES  (6U)
#define BALL_POSITION_CAPTURE_MAX_SPAN_MM (5.0f)
#define BALL_POSITION_CAPTURE_LIMIT_MM    (110.0f)

typedef enum
{
    BALL_POSITION_CAPTURE_WAITING = 0,
    BALL_POSITION_CAPTURE_LOCKED
} ball_position_capture_result_t;

typedef struct
{
    uint32_t start_ms;
    uint32_t last_frame_count;
    uint32_t elapsed_ms;
    uint16_t sample_count;
    uint8_t active;
    float sum_mm;
    float minimum_mm;
    float maximum_mm;
    float mean_mm;
    float span_mm;
} ball_position_capture_t;
```

`ball_position_capture_push()` 必须按以下顺序处理：忽略重复帧；越界时完整复位并记住已见帧号；首帧开启窗口；新样本导致跨度超过 5 mm 时以该样本重启窗口；更新均值、跨度和经过时间；达到 300 ms 且不少于 6 帧时返回 `LOCKED`。

- [ ] **Step 4: 运行捕获器测试**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl tests/ball_position_capture_test.c Control/ball_position_capture.c -lm -o tests/ball_position_capture_test.exe
rtk .\tests\ball_position_capture_test.exe
```

Expected: 编译成功，测试进程退出码为 0。

- [ ] **Step 5: 检查格式和工作区边界**

Run:

```powershell
rtk git diff --check -- Control/ball_position_capture.h Control/ball_position_capture.c tests/ball_position_capture_test.c
rtk git status --short
```

Expected: 新文件无空白错误；既有暂存文件保持不变。测试生成的 `.exe` 不加入 Git。

---

### Task 2: 扩展一圈状态机支持 A 点锁时和附加 1 m

**Files:**
- Create: `tests/straight_turn_post_lap_contract_test.c`
- Modify: `Control/straight_turn_test.h`
- Modify: `Control/straight_turn_test.c`

**Interfaces:**
- Consumes: 原有 IMU、四路红外、编码器轮速和 `Get_Target_Encoder()`。
- Produces:
  - 新状态 `STRAIGHT_TURN_POST_LAP`、`STRAIGHT_TURN_BRAKING`。
  - `volatile uint32_t StraightTurnLapTimeMs;`
  - `volatile float StraightTurnPostLapDistanceM;`
  - `void StraightTurnTest_StartWithPostLap(float target_speed_mps, float acceleration_mps2, float post_lap_distance_m);`
  - 原 `StraightTurnTest_Start(float, float)` 保留并以 `post_lap_distance_m = 0.0f` 调用扩展接口。

- [ ] **Step 1: 写头文件契约失败测试**

```c
#include "straight_turn_test.h"

static void (*start_with_post_lap)(float, float, float) =
    StraightTurnTest_StartWithPostLap;

int main(void)
{
    return (STRAIGHT_TURN_POST_LAP == STRAIGHT_TURN_BRAKING) ||
           (start_with_post_lap == 0);
}
```

- [ ] **Step 2: 运行契约测试并确认失败**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/straight_turn_post_lap_contract_test.c
```

Expected: FAIL，提示新状态或 `StraightTurnTest_StartWithPostLap` 尚未声明。

- [ ] **Step 3: 扩展公共状态和启动接口**

在 `STRAIGHT_TURN_ARC_2` 后加入：

```c
STRAIGHT_TURN_POST_LAP,
STRAIGHT_TURN_BRAKING,
```

增加：

```c
extern volatile uint32_t StraightTurnLapTimeMs;
extern volatile float StraightTurnPostLapDistanceM;

void StraightTurnTest_StartWithPostLap(
    float target_speed_mps,
    float acceleration_mps2,
    float post_lap_distance_m);
```

原两参数启动函数必须保留。

- [ ] **Step 4: 运行契约测试确认通过**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/straight_turn_post_lap_contract_test.c
```

Expected: PASS。

- [ ] **Step 5: 实现第二个半圆后的分流**

增加内部配置和制动常量：

```c
#define STRAIGHT_TURN_POST_LAP_MAX_DISTANCE_M (2.0f)
#define STRAIGHT_TURN_POST_DECELERATION_MPS2  (0.20f)
#define STRAIGHT_TURN_STOP_SPEED_MPS          (0.01f)

static float straight_turn_post_lap_target_m;
static float straight_turn_post_lap_heading_yaw;
```

启动时将附加距离限制在 `0.0f..2.0f`。第二个半圆在附加距离大于零时使用非停车的连续圆弧启动路径；圆弧完成时锁存：

```c
StraightTurnLapTimeMs = StraightTurnElapsedMs;
StraightTurnPostLapDistanceM = 0.0f;
straight_turn_post_lap_heading_yaw = straight_turn_last_valid_yaw;
StraightTurnState = STRAIGHT_TURN_POST_LAP;
```

附加距离为零时保持原 `DONE` 和停车行为。

- [ ] **Step 6: 实现附加循迹和制动**

`POST_LAP` 每 5 ms 更新 IMU、红外和正向编码器平均距离，使用 `straight_turn_heading_omega(straight_turn_post_lap_heading_yaw, filtered_line_error)` 继续循迹。距离达到配置值后进入 `BRAKING`。

`BRAKING` 每周期执行：

```c
StraightTurnCommandSpeed -=
    STRAIGHT_TURN_POST_DECELERATION_MPS2 *
    STRAIGHT_TURN_CONTROL_PERIOD_S;
```

速度降到 `STRAIGHT_TURN_STOP_SPEED_MPS` 以下时，将左右目标速度清零、状态置为 `DONE`、`Flag_Stop = 1U`。制动阶段仍使用红外和航向控制，不能直线开环滑行。

- [ ] **Step 7: 重置所有新增状态并做静态检查**

`StraightTurnTest_Reset()` 必须清零成绩、附加距离和内部配置；`StraightTurnTest_Stop()` 必须清零命令速度并安全停车。

Run:

```powershell
rtk git diff --check -- Control/straight_turn_test.h Control/straight_turn_test.c tests/straight_turn_post_lap_contract_test.c
rtk rg -n "StraightTurnLapTimeMs|StraightTurnPostLapDistanceM|STRAIGHT_TURN_POST_LAP|STRAIGHT_TURN_BRAKING" Control/straight_turn_test.h Control/straight_turn_test.c
```

Expected: 新状态在声明、复位、运行分发和 OLED 可见变量中均有对应处理。

---

### Task 3: 新增 BALL HOLD LAP 协调任务

**Files:**
- Create: `Control/ball_hold_lap.h`
- Create: `Control/ball_hold_lap.c`
- Create: `tests/ball_hold_lap_contract_test.c`

**Interfaces:**
- Consumes:
  - `vision_ball_position_mm`、`vision_ball_frame_count`、`vision_ball_last_update_ms`、`vision_ball_position_valid`。
  - `ball_position_capture_*`。
  - `ball_balance_set_reference()`。
  - `StraightTurnTest_StartWithPostLap()`、`StraightTurnState`、`StraightTurnFault`、成绩和附加里程。
- Produces:
  - `void ball_hold_lap_init(void);`
  - `void ball_hold_lap_reset(void);`
  - `void ball_hold_lap_start(void);`
  - `void ball_hold_lap_stop(void);`
  - `void ball_hold_lap_update(void);`
  - `uint8_t ball_hold_lap_controller_enabled(void);`
  - OLED 所需目标、当前位置、实时/最大误差、捕获统计、成绩合格标志和任务状态。

- [ ] **Step 1: 写任务公共接口契约测试**

```c
#include "ball_hold_lap.h"

static void (*start_task)(void) = ball_hold_lap_start;
static void (*update_task)(void) = ball_hold_lap_update;

int main(void)
{
    return (BALL_HOLD_LAP_READY == BALL_HOLD_LAP_DONE) ||
           start_task == 0 || update_task == 0;
}
```

- [ ] **Step 2: 运行契约测试并确认失败**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/ball_hold_lap_contract_test.c
```

Expected: FAIL，提示 `ball_hold_lap.h` 不存在。

- [ ] **Step 3: 定义任务状态和只读快照变量**

```c
typedef enum
{
    BALL_HOLD_LAP_READY = 0,
    BALL_HOLD_LAP_CAPTURING,
    BALL_HOLD_LAP_RUNNING,
    BALL_HOLD_LAP_POST_LAP,
    BALL_HOLD_LAP_BRAKING,
    BALL_HOLD_LAP_DONE,
    BALL_HOLD_LAP_ABORTED,
    BALL_HOLD_LAP_FAULT
} ball_hold_lap_state_t;
```

公开的 `volatile` 快照至少包括：

```c
ball_hold_lap_state_t ball_hold_lap_state;
float ball_hold_lap_target_mm;
float ball_hold_lap_current_mm;
float ball_hold_lap_error_mm;
float ball_hold_lap_max_abs_error_mm;
float ball_hold_lap_capture_mean_mm;
float ball_hold_lap_capture_span_mm;
uint16_t ball_hold_lap_capture_frames;
uint32_t ball_hold_lap_capture_elapsed_ms;
uint32_t ball_hold_lap_time_ms;
uint8_t ball_hold_lap_time_pass;
uint8_t ball_hold_lap_position_pass;
uint8_t ball_hold_lap_fault;
```

- [ ] **Step 4: 运行接口契约测试确认通过**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/ball_hold_lap_contract_test.c
```

Expected: PASS。

- [ ] **Step 5: 实现捕获与启动**

`ball_hold_lap_start()` 必须重置成绩、捕获器和跑道状态，保持 `Flag_Stop = 1U`，进入 `CAPTURING`。`ball_hold_lap_update()` 在视觉有效且年龄不超过 200 ms 时读取一致快照并调用捕获器；捕获器返回 `LOCKED` 时：

```c
ball_hold_lap_target_mm = target_mm;
ball_balance_set_reference(target_mm, 0.0f);
StraightTurnTest_StartWithPostLap(
    STRAIGHT_TURN_BALL_SPEED_MPS,
    STRAIGHT_TURN_BALL_ACCELERATION_MPS2,
    1.00f);
ball_hold_lap_state = BALL_HOLD_LAP_RUNNING;
```

捕获无效时调用 `ball_position_capture_reset()` 并继续等待。

- [ ] **Step 6: 实现运行镜像、误差和结果锁存**

仅对新鲜且新的视觉帧更新：

```c
error_mm = vision_ball_position_mm - ball_hold_lap_target_mm;
abs_error_mm = (error_mm < 0.0f) ? -error_mm : error_mm;
```

更新最大误差；超过 10 mm 时将 `ball_hold_lap_position_pass` 锁存为 0。根据 `StraightTurnState` 映射 `RUNNING`、`POST_LAP`、`BRAKING`、`DONE` 或 `FAULT`。首次观察到 `StraightTurnLapTimeMs != 0U` 时锁存时间和 `<=30000U` 合格标志。

- [ ] **Step 7: 实现停止和闭环使能策略**

`ball_hold_lap_controller_enabled()` 只在 `RUNNING`、`POST_LAP` 和 `BRAKING` 返回 1。`ball_hold_lap_stop()` 调用 `StraightTurnTest_Stop()`，将活动任务置为 `ABORTED`；初始化/复位和完成后的关闭路径必须让调度器禁用 `ball_balance`，由现有控制器回中逻辑输出 `SERVO_NEUTRAL_PULSE_US`。

- [ ] **Step 8: 检查任务状态覆盖**

Run:

```powershell
rtk git diff --check -- Control/ball_hold_lap.h Control/ball_hold_lap.c tests/ball_hold_lap_contract_test.c
rtk rg -n "BALL_HOLD_LAP_(READY|CAPTURING|RUNNING|POST_LAP|BRAKING|DONE|ABORTED|FAULT)" Control/ball_hold_lap.c Control/ball_hold_lap.h
```

Expected: 每个状态至少出现在声明、更新或显示数据路径中，无遗漏分支。

---

### Task 4: 调度、按键和 OLED 分页集成

**Files:**
- Modify: `Control/control.h`
- Modify: `Control/control.c`
- Modify: `Control/show.c`
- Modify: `empty.c`
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: Task 2 和 Task 3 的全部公共接口。
- Produces:
  - `RUN_MODE_BALL_HOLD_LAP`。
  - 六项用户菜单表和 `Menu_SelectionIndex`。
  - 每页 4 项的自动分页菜单。
  - `BALL HOLD` 捕获、运行、附加里程、制动和结果页面。

- [ ] **Step 1: 将新模式加入运行模式定义**

在现有最大模式编号之后加入：

```c
#define RUN_MODE_BALL_HOLD_LAP 8
#define RUN_MODE_COUNT         9
#define MENU_MODE_COUNT        6U
#define MENU_ITEMS_PER_PAGE    4U
```

定义共享菜单项：

```c
typedef struct
{
    uint8_t mode;
    const char *label;
} menu_item_t;

extern const menu_item_t Menu_Items[MENU_MODE_COUNT];
extern volatile uint8_t Menu_SelectionIndex;
```

- [ ] **Step 2: 用数据表替换硬编码菜单轮转**

在 `control.c` 定义：

```c
const menu_item_t Menu_Items[MENU_MODE_COUNT] =
{
    {RUN_MODE_STRAIGHT_TURN, "ONE LAP"},
    {RUN_MODE_BALL_LAP, "BALL LAP"},
    {RUN_MODE_BALL_HOLD_LAP, "BALL HOLD"},
    {RUN_MODE_BALL_STATIC, "STATIC HYB"},
    {RUN_MODE_SERVO_ADJUST, "SERVO ADJ"},
    {RUN_MODE_IMU_DEBUG, "IMU DEBUG"}
};
```

菜单单击执行索引模 6 加一并同步 `Menu_Selection`；双击按当前表项进入。长按返回菜单时通过线性扫描恢复当前模式对应索引，未找到则回到 `RUN_MODE_MENU_DEFAULT`。

- [ ] **Step 3: 将新任务接入 5 ms 调度**

模式改变时调用 `ball_hold_lap_reset()`。在钢球闭环使能之前调用 `ball_hold_lap_update()`。新模式闭环使能条件为 `ball_hold_lap_controller_enabled()`；车辆启动加速度前馈与原 `BALL LAP` 一样读取 `StraightTurnStartupAccelerationMps2`。新模式必须进入 `StraightTurnTest_Run()` 分支。

保持现有执行顺序：

```text
读取按键/编码器
-> 模式复位
-> BALL HOLD 任务更新
-> 设置钢球闭环使能及加速度前馈
-> ball_balance_update
-> Flag_Stop 安全返回
-> 跑道状态机
-> 轮速 PI 和电机 PWM
```

- [ ] **Step 4: 接入按键启动/停止**

新模式单击行为：

```c
if (ball_hold_lap_state == BALL_HOLD_LAP_READY ||
    ball_hold_lap_state == BALL_HOLD_LAP_DONE ||
    ball_hold_lap_state == BALL_HOLD_LAP_ABORTED ||
    ball_hold_lap_state == BALL_HOLD_LAP_FAULT)
{
    ball_hold_lap_start();
}
else
{
    ball_hold_lap_stop();
}
```

长按返回菜单前必须调用 `ball_hold_lap_stop()`。

- [ ] **Step 5: 初始化并加入 Keil 工程**

`empty.c` 在 `ball_balance_init()` 后调用 `ball_hold_lap_init()`。在 Keil `Control` 组中加入：

```xml
<File>
  <FileName>ball_position_capture.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\ball_position_capture.c</FilePath>
</File>
<File>
  <FileName>ball_hold_lap.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\ball_hold_lap.c</FilePath>
</File>
```

- [ ] **Step 6: 实现每页 4 项菜单**

`menu_oled_show()` 计算：

```c
uint8_t page = Menu_SelectionIndex / MENU_ITEMS_PER_PAGE;
uint8_t first = page * MENU_ITEMS_PER_PAGE;
uint8_t end = first + MENU_ITEMS_PER_PAGE;
```

将 `end` 限制到 `MENU_MODE_COUNT`，在 y=`0,12,24,36` 显示本页条目，y=`52` 显示 `P1/2  1>N 2>OK`。页数采用向上取整 `(MENU_MODE_COUNT + MENU_ITEMS_PER_PAGE - 1U) / MENU_ITEMS_PER_PAGE`。

- [ ] **Step 7: 实现 BALL HOLD 状态页面**

新增 `ball_hold_lap_oled_show()`，按状态显示：

- `READY`：视觉位置或 `NO VISION`、`1:START`。
- `CAPTURING`：毫秒进度、帧数、均值、跨度。
- `RUNNING`：`StraightTurnElapsedMs`、目标、当前位置、误差。
- `POST_LAP`：锁存的一圈时间和 `StraightTurnPostLapDistanceM`。
- `BRAKING`：锁存成绩和 `BRAKE`。
- `DONE`：成绩 `PASS/OVERTIME`、目标、最大误差和位置 `PASS/FAIL`。
- `ABORTED/FAULT`：明确状态，未完成时不显示伪成绩。

所有浮点显示复用 `show.c` 现有的定点格式化辅助函数。

- [ ] **Step 8: 静态检查集成完整性**

Run:

```powershell
rtk rg -n "RUN_MODE_BALL_HOLD_LAP" Control/control.h Control/control.c Control/show.c
rtk rg -n "ball_position_capture.c|ball_hold_lap.c" keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
rtk rg -n "\b1250\b|\b1260\b" Control Hardware empty.c -g "*.c" -g "*.h"
rtk git diff --check -- Control/control.h Control/control.c Control/show.c empty.c keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

Expected: 新模式覆盖调度、按键、显示和工程文件；除 `SERVO_NEUTRAL_PULSE_US (1250U)` 定义外无控制路径写死中位值。

---

### Task 5: 全量验证与板上验收清单

**Files:**
- Modify only if verification exposes defects in Tasks 1–4.

**Interfaces:**
- Consumes: 完整固件。
- Produces: 可烧录 AXF 和明确的板上验证记录。

- [ ] **Step 1: 重跑所有主机测试**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl tests/ball_position_capture_test.c Control/ball_position_capture.c -lm -o tests/ball_position_capture_test.exe
rtk .\tests\ball_position_capture_test.exe
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/straight_turn_post_lap_contract_test.c
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -fsyntax-only tests/ball_hold_lap_contract_test.c
```

Expected: 全部退出码为 0。

- [ ] **Step 2: 执行 Keil 强制重建**

Run:

```powershell
rtk 'D:\Infineon\Keli\Keil_v5\UV4\UV4.exe' -r keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -t MSPM0G3507_Project
```

Expected: `0 Error(s), 0 Warning(s)`，产物位于 `keil/Objects/`。构建产物保留在工作区但不加入实现提交。

- [ ] **Step 3: 检查最终差异和暂存边界**

Run:

```powershell
rtk git diff --check
rtk git status --short
rtk git diff --cached --name-only
rtk git diff -- Control/ball_position_capture.h Control/ball_position_capture.c Control/ball_hold_lap.h Control/ball_hold_lap.c Control/straight_turn_test.h Control/straight_turn_test.c Control/control.h Control/control.c Control/show.c empty.c keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

Expected: 仅包含本功能和用户原有参数调整；用户预先暂存的文件仍保持暂存，测试 `.exe` 未暂存。

- [ ] **Step 4: 板上测试中心、正偏和负偏目标**

对三个初始位置各执行一次：

1. 进入 `BALL HOLD`。
2. 将钢球静置，单击启动。
3. 确认至少等待约 300 ms 后才起步。
4. 记录锁存目标、最大误差和一圈时间。

Expected: 锁存目标跟随三个不同初始位置；一圈时间 `<=30.000 s`；最大有效视觉误差 `<=10 mm`。

- [ ] **Step 5: 板上测试不稳定捕获和视觉恢复**

捕获期间持续移动钢球超过 5 mm，确认车辆保持等待。运行时断开视觉数据至少 250 ms，确认车辆继续循迹且舵机回到 `SERVO_NEUTRAL_PULSE_US`；恢复数据后确认闭环无明显瞬时冲击。

- [ ] **Step 6: 板上测试 A 点、附加距离和菜单回归**

确认第二个半圆完成时 OLED 一圈时间停止增长；附加距离达到 1.00 m 后进入 `BRAKING`；最终停车后成绩不包含附加段。逐项进入六个菜单模式，确认跨页、双击进入和长按返回均正确。

- [ ] **Step 7: 交付时报告未完成的硬件验证**

如果当前环境未连接目标板，只声明主机测试和 Keil 构建结果，并把 Steps 4–6 明确列为待用户上板验证；不得把未执行的板上测试声称为通过。

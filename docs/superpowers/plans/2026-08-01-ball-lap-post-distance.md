# BALL LAP 一圈后附加直行 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修改 `RUN_MODE_BALL_LAP`，使车辆完成一圈后继续沿直道循迹 1.00 m，再平滑减速停车。

**Architecture:** 复用 `Control/straight_turn_test.c` 已有的可配置附加段状态机。只在 `Control/control.c` 的 BALL LAP 启动入口传入 1.00 m，并在 `Control/straight_turn_test.h` 提供命名常量；普通 `ONE LAP` 继续使用零附加距离的原入口，其他模式不变。

**Tech Stack:** C99、TI MSPM0 DriverLib、ARMCLANG/Keil uVision、现有编码器速度换算和 5 ms 定时控制循环。

## Global Constraints

- 一圈判定仍为现有跑道状态机完成第二个半圆。
- 附加直行距离从一圈完成时清零，使用左右轮编码器平均里程累计 1.00 m。
- 达到 1.00 m 后进入已有 `BRAKING` 状态并置 `Flag_Stop` 停车。
- `ONE LAP`、`BALL HOLD LAP`、`BALL STATIC` 及其他运行模式行为保持不变。
- 不修改 `empty.syscfg`、生成配置文件、`source/`、TI DriverLib 或硬件驱动。
- 不提交 `keil/Objects/`、uVision 用户设置或其他构建产物。

---

## 文件结构与职责

- Modify: `Control/straight_turn_test.h` — 增加 BALL LAP 的 1.00 m 附加距离命名常量。
- Modify: `Control/control.c:445-463` — 在 BALL LAP 按键启动分支选择附加距离启动接口。
- Test/verify: `Control/straight_turn_test.c` — 不改代码；核验既有 `POST_LAP -> BRAKING -> DONE` 实现被正确调用。
- Test/verify: Keil 工程 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` — 执行目标构建。

## Task 1: Route BALL LAP through the existing post-lap distance state

**Files:**
- Modify: `Control/straight_turn_test.h`
- Modify: `Control/control.c:445-463`

**Interfaces:**
- Consumes: `StraightTurnTest_Start()`, `StraightTurnTest_StartWithPostLap()`, `RUN_MODE_BALL_LAP`, `STRAIGHT_TURN_BALL_SPEED_MPS`, and `STRAIGHT_TURN_BALL_ACCELERATION_MPS2`.
- Produces: `STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M` with value `1.00f`, and a BALL LAP startup call that stores this distance in `straight_turn_post_lap_target_m`.

- [ ] **Step 1: Confirm the current baseline and exact edit sites**

  Run:

  ```powershell
  rtk git status --short
  rtk rg -n "STRAIGHT_TURN_BALL_SPEED_MPS|StraightTurnTest_Start\(|Run_Mode == RUN_MODE_STRAIGHT_TURN|Run_Mode == RUN_MODE_BALL_LAP" Control\straight_turn_test.h Control\control.c
  ```

  Expected: the worktree contains only the already committed design/plan history, and the existing shared branch currently calls `StraightTurnTest_Start()` for both `ONE LAP` and `BALL LAP`.

- [ ] **Step 2: Add the named 1.00 m constant**

  In `Control/straight_turn_test.h`, immediately after the BALL speed/acceleration constants, add:

  ```c
  #define STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M (1.00f)
  ```

  Keep the existing naming and four-space indentation style; do not add a second copy of the distance in `control.c`.

- [ ] **Step 3: Select the post-lap startup API for BALL LAP only**

  In the `Key()` branch that handles `RUN_MODE_STRAIGHT_TURN || RUN_MODE_BALL_LAP`, replace the single shared start call with this mode-specific branch:

  ```c
  if (Run_Mode == RUN_MODE_BALL_LAP)
  {
      StraightTurnTest_StartWithPostLap(
          STRAIGHT_TURN_BALL_SPEED_MPS,
          STRAIGHT_TURN_BALL_ACCELERATION_MPS2,
          STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M);
  }
  else
  {
      StraightTurnTest_Start(
          STRAIGHT_TURN_FAST_SPEED_MPS,
          STRAIGHT_TURN_FAST_ACCELERATION_MPS2);
  }
  ```

  Preserve the surrounding `Reset_Velocity_PI()` call and the existing stop branch. This ensures `ONE LAP` keeps its original zero-distance completion while BALL LAP enters `STRAIGHT_TURN_POST_LAP` after `ARC_2`.

- [ ] **Step 4: Run focused static checks**

  Run:

  ```powershell
  rtk git diff --check
  rtk rg -n -C 5 "STRAIGHT_TURN_BALL_POST_LAP_DISTANCE_M|StraightTurnTest_StartWithPostLap|StraightTurnTest_Start\(" Control\straight_turn_test.h Control\control.c
  ```

  Expected: the constant is defined once; BALL LAP calls `StartWithPostLap()` with the constant; the non-BALL `STRAIGHT_TURN` branch calls `Start()`; no whitespace errors are reported.

- [ ] **Step 5: Commit the focused code change**

  ```powershell
  rtk git add Control\straight_turn_test.h Control\control.c
  rtk git commit -m "ball: drive 1m after BALL LAP"
  ```

## Task 2: Verify the state-machine path and firmware build

**Files:**
- Test/verify: `Control/straight_turn_test.c`
- Test/verify: `Control/control.c`
- Test/verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: `StraightTurnTest_StartWithPostLap(..., 1.00f)` from Task 1.
- Produces: evidence that `ARC_2` records `StraightTurnLapTimeMs`, `POST_LAP` accumulates distance, `BRAKING` reduces speed, and `DONE` sets `Flag_Stop`.

- [ ] **Step 1: Inspect the existing completion path**

  Run:

  ```powershell
  rtk rg -n -C 8 "StraightTurnLapTimeMs|StraightTurnPostLapDistanceM|STRAIGHT_TURN_POST_LAP|STRAIGHT_TURN_BRAKING|Flag_Stop = 1U" Control\straight_turn_test.c
  ```

  Expected: after the second arc, `StraightTurnLapTimeMs` is latched and a positive post-lap target selects `STRAIGHT_TURN_POST_LAP`; reaching the target selects `STRAIGHT_TURN_BRAKING`; the braking endpoint selects `STRAIGHT_TURN_DONE` and sets `Flag_Stop`.

- [ ] **Step 2: Build the Keil target**

  ```powershell
  rtk proxy powershell -NoProfile -Command "& 'D:\Infineon\Keli\Keil_v5\UV4\UV4.exe' -b 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project'"
  ```

  Expected: Keil exits successfully and produces the normal firmware output under `keil\Objects\` without new compile errors or warnings.

- [ ] **Step 3: Check the final worktree scope**

  ```powershell
  rtk git status --short
  rtk git log -2 --oneline
  ```

  Expected: only the two intended source files are part of the feature commit; ignored Keil output is not staged.

- [ ] **Step 4: Record hardware verification requirements**

  On the target board, run `BALL LAP` and confirm: the car completes one lap, continues along the straight for approximately 1 m, then decelerates and stops. Also run `ONE LAP` once and confirm it still stops at the original one-lap completion point. Record actual board, firmware mode, and observed motor/serial/OLED behavior for the handoff.

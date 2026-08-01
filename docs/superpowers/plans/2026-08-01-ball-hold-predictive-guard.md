# Ball Hold Predictive Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce `BALL HOLD` startup and occasional turn disturbances by fusing camera velocity and applying a continuous predicted-position recovery velocity before the ball reaches the `+/-10 mm` acceptance boundary.

**Architecture:** Preserve the current motion-gated startup feedforward and cascade controller. Add low-weight camera-velocity fusion to the existing observer, calculate the predictive guard in a pure module, and let `ball_balance` select the stronger center-seeking velocity only while `BALL HOLD` explicitly enables the guard.

**Tech Stack:** C99, MSPM0G3507 bare-metal firmware, host-side GCC assertion tests, ARMCLANG/Keil uVision.

## Global Constraints

- Do not calculate or compensate turn centripetal acceleration.
- Do not change the vehicle straight, turn, or braking trajectory.
- Do not reduce vehicle speed based on ball position in the first version.
- Do not change `BALL STATIC` point-to-point behavior.
- Keep the existing position PI, velocity loop, position feedforward, distance limiter, `500..2200 us` servo range, and `5 us` quantization.
- Keep `BALL_BALANCE_DISTANCE_PROFILE_ACCEL_MM_S2=70.0f`.
- Keep the startup gate at `0.02 m/s` for two consecutive `5 ms` ticks.
- Use `camera_velocity_blend=0.25`, `prediction_time=0.060 s`, `soft_boundary=5.0 mm`, `guard_gain=8.0 /s`, and `guard_velocity_max=60.0 mm/s`.
- Do not add position D, another integrator, latch, creep state, minimum approach speed, or pre-motion servo preload.
- Do not stage or commit any file.

---

## File Map

- `Control/ball_state_observer.c/.h`: fuse camera velocity only on new frames and publish fusion diagnostics.
- `Control/ball_predictive_guard.c/.h`: pure predicted-error and recovery-velocity calculation.
- `Control/ball_balance.c/.h`: enable/disable the guard, merge it with the existing target velocity, freeze integration when it overrides, and publish controller diagnostics.
- `Control/control.c`: enable the guard only while `BALL HOLD` owns the ball controller.
- `Control/straight_turn_test.c/.h`: publish the existing startup-motion-gate state without changing its behavior.
- `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`: add the new predictive-guard source to the firmware target.
- `tools/host_tests/test_ball_state_observer.c`: observer velocity-fusion regression tests.
- `tools/host_tests/test_ball_predictive_guard.c`: pure guard tests.
- `tools/host_tests/test_ball_balance.c`: guard/controller integration and current `a_profile=70` regression tests.
- `tools/host_tests/test_straight_turn_timing.c`: startup-gate diagnostic regression test.

---

### Task 1: Restore the Current `a_profile=70` Host-Test Baseline

**Files:**
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- Consumes: `ball_braking_calculate_distance_velocity_limit()` and the existing `ball_balance` diagnostics.
- Produces: a green controller baseline whose expected values match the already-selected `70 mm/s^2` profile.

- [ ] **Step 1: Compile and run the current controller test**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe

rtk tools/host_tests/test_ball_balance.exe
```

Expected: compilation succeeds, then execution fails because the test still expects the old `a_profile=30` limits such as `40.66457 mm/s`.

- [ ] **Step 2: Replace the stale distance-limit scenarios**

Use these literal `a_profile=70`, `T=0.060 s` expectations:

```text
error=50 mm -> distance limit=79.57136 mm/s
error=40 mm -> distance limit=70.75092 mm/s
error=35 mm -> distance limit=65.92589 mm/s
error=30 mm -> distance limit=60.74336 mm/s
error=20 mm -> distance limit=48.88145 mm/s
error=15 mm -> distance limit=41.81782 mm/s
error=10 mm -> distance limit=33.45156 mm/s
error=4 mm  -> distance limit=19.83414 mm/s
```

Retain both limited and non-limited cases:

```c
/* Limited: v_pid=100 is above the 50 mm distance limit. */
ball_state_observer.position_mm = 0.0f;
ball_state_observer.velocity_mm_s = 60.0f;
ball_balance_set_reference(50.0f, 0.0f);
ball_balance_update();
assert_close(ball_balance_position_pid_velocity_mm_s, 100.0f);
assert_close(
    ball_balance_distance_velocity_limit_mm_s,
    79.57136f);
assert_close(ball_balance_target_velocity_mm_s, 79.57136f);
assert(ball_balance_distance_limited == 1U);

/* Not limited: v_pid=60 is below the 30 mm distance limit. */
ball_state_observer.position_mm = 20.0f;
ball_state_observer.velocity_mm_s = 60.0f;
ball_balance_set_reference(50.0f, 0.0f);
ball_balance_update();
assert_close(ball_balance_position_pid_velocity_mm_s, 60.0f);
assert_close(
    ball_balance_distance_velocity_limit_mm_s,
    60.74336f);
assert_close(ball_balance_target_velocity_mm_s, 60.0f);
assert(ball_balance_distance_limited == 0U);
```

Use the `50 mm` limited case for the repeated integral-freeze test. After the
repeated updates, move to an error of `4 mm` and expect `8.02 mm/s`: `8.0`
comes from position P and `0.02` is the first allowed integral candidate on
the current non-limited tick.

- [ ] **Step 3: Recompile and verify the baseline**

Run the Step 1 compile and executable commands again.

Expected: both exit `0` with no warning.

---

### Task 2: Fuse Camera Velocity on New Vision Frames

**Files:**
- Modify: `Control/ball_state_observer.h`
- Modify: `Control/ball_state_observer.c`
- Modify: `tools/host_tests/test_ball_state_observer.c`
- Test: `tools/host_tests/test_ball_state_observer.c`

**Interfaces:**
- Consumes: `ball_vision_measurement_t.velocity_mm_s`, new-frame detection, position residual, and frame interval.
- Produces: `BALL_OBSERVER_CAMERA_VELOCITY_BLEND`, `camera_velocity_mm_s`, `velocity_blend_active`, and the fused `observer->velocity_mm_s`.

- [ ] **Step 1: Write RED observer assertions**

Extend `ball_state_observer_t` expectations in the test:

```c
assert_close(observer.camera_velocity_mm_s, 0.0f);
assert(observer.velocity_blend_active == 0U);
```

For the existing second visual frame, change the expected values to:

```c
assert_close(observer.position_mm, 3.1f);
assert_close(observer.velocity_mm_s, 107.5f);
assert_close(observer.camera_velocity_mm_s, 100.0f);
assert(observer.velocity_blend_active == 1U);
assert_close(observer.acceleration_mm_s2, 300.0f);

ball_state_observer_update(&observer, &measurement, 25U);
assert_close(observer.position_mm, 3.6375f);
assert_close(observer.velocity_mm_s, 107.5f);
assert(observer.velocity_blend_active == 0U);
assert_close(observer.acceleration_mm_s2, 240.0f);
```

The `107.5` expectation is:

```text
v_residual = 100 + 0.10 * 2 mm / 0.020 s = 110 mm/s
v_fused    = 0.75 * 110 + 0.25 * 100 = 107.5 mm/s
```

Add a separate observer instance with:

```c
measurement.position_mm = 0.0f;
measurement.velocity_mm_s = 0.0f;
measurement.frame_count = 1U;
measurement.sample_ms = 0U;

/* Initialize at zero, then deliver a new frame 20 ms later. */
measurement.position_mm = 2.0f;
measurement.velocity_mm_s = 50.0f;
measurement.frame_count = 2U;
measurement.sample_ms = 20U;
```

Expect `position=1.1 mm`, `v_residual=10 mm/s`, and
`v_fused=20 mm/s`. Replay frame `2` and verify the camera velocity is not
fused a second time. Supply `NAN` as camera velocity on a later new frame and
verify the finite position-residual estimate remains valid while
`velocity_blend_active==0`.

- [ ] **Step 2: Compile to verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe
```

Expected: compilation fails because the blend constant and diagnostic fields do not exist.

- [ ] **Step 3: Add the observer fields and constant**

In `Control/ball_state_observer.h` add:

```c
#define BALL_OBSERVER_CAMERA_VELOCITY_BLEND (0.25f)

/* Inside ball_state_observer_t. */
volatile float camera_velocity_mm_s;
volatile uint8_t velocity_blend_active;
```

Reset both fields in `ball_state_observer_reset()` and
`invalidate_dynamic_state()`. At the start of each valid update set
`velocity_blend_active=0U`, so it indicates fusion on the current control tick
only.

- [ ] **Step 4: Implement finite camera-velocity fusion**

Add the existing finite-value pattern:

```c
static uint8_t finite_float(float value)
{
    return
        (value == value &&
         value <= 3.402823466e+38F &&
         value >= -3.402823466e+38F)
            ? 1U : 0U;
}
```

In the initialized new-frame branch, replace the direct residual-only velocity
write with:

```c
float residual_velocity_mm_s;
float camera_velocity_mm_s;

residual_velocity_mm_s = clamp_float(
    observer->velocity_mm_s +
        BALL_OBSERVER_BETA * residual_mm *
            1000.0f / (float)frame_dt_ms,
    -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
    BALL_OBSERVER_VELOCITY_LIMIT_MM_S);

if (finite_float(measurement->velocity_mm_s) != 0U)
{
    camera_velocity_mm_s = clamp_float(
        measurement->velocity_mm_s,
        -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
        BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
    observer->camera_velocity_mm_s = camera_velocity_mm_s;
    observer->velocity_mm_s = clamp_float(
        (1.0f - BALL_OBSERVER_CAMERA_VELOCITY_BLEND) *
                residual_velocity_mm_s +
            BALL_OBSERVER_CAMERA_VELOCITY_BLEND *
                camera_velocity_mm_s,
        -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
        BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
    observer->velocity_blend_active = 1U;
}
else
{
    observer->camera_velocity_mm_s = 0.0f;
    observer->velocity_mm_s = residual_velocity_mm_s;
}
```

During first-frame initialization, use:

```c
if (finite_float(measurement->velocity_mm_s) != 0U)
{
    observer->velocity_mm_s = clamp_float(
        measurement->velocity_mm_s,
        -BALL_OBSERVER_VELOCITY_LIMIT_MM_S,
        BALL_OBSERVER_VELOCITY_LIMIT_MM_S);
    observer->camera_velocity_mm_s =
        observer->velocity_mm_s;
}
else
{
    observer->velocity_mm_s = 0.0f;
    observer->camera_velocity_mm_s = 0.0f;
}
observer->velocity_blend_active = 0U;
```

This prevents a non-finite first-frame camera velocity from entering the
observer while preserving the current finite-value initialization behavior.

- [ ] **Step 5: Compile and verify GREEN**

Run the Step 2 compile command and:

```powershell
rtk tools/host_tests/test_ball_state_observer.exe
```

Expected: both exit `0` with no warning.

---

### Task 3: Add the Pure Predictive-Guard Calculation

**Files:**
- Create: `Control/ball_predictive_guard.h`
- Create: `Control/ball_predictive_guard.c`
- Create: `tools/host_tests/test_ball_predictive_guard.c`
- Test: `tools/host_tests/test_ball_predictive_guard.c`

**Interfaces:**
- Consumes: current position error, fused ball velocity, prediction time, soft boundary, guard gain, and guard velocity maximum.
- Produces:

```c
typedef struct
{
    float predicted_error_mm;
    float guard_velocity_mm_s;
    uint8_t active;
} ball_predictive_guard_result_t;

uint8_t ball_predictive_guard_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float prediction_time_s,
    float soft_boundary_mm,
    float guard_gain_per_s,
    float guard_velocity_max_mm_s,
    ball_predictive_guard_result_t *result);
```

- [ ] **Step 1: Write the pure RED test**

Create `tools/host_tests/test_ball_predictive_guard.c` with assertions for:

```c
/* Centered and stationary. */
calculate(0.0f, 0.0f, 0.060f, 5.0f, 8.0f, 60.0f);
/* predicted=0, guard=0, active=0 */

/* Ball is +6 mm from target and moving farther positive at 50 mm/s.
 * Controller error convention is target-position, hence error=-6. */
calculate(-6.0f, 50.0f, 0.060f, 5.0f, 8.0f, 60.0f);
/* predicted=-9, guard=-32, active=1 */

/* Same position but returning toward center. */
calculate(-6.0f, -50.0f, 0.060f, 5.0f, 8.0f, 60.0f);
/* predicted=-3, guard=0, active=0 */

/* Opposite-direction symmetry. */
calculate(6.0f, -50.0f, 0.060f, 5.0f, 8.0f, 60.0f);
/* predicted=+9, guard=+32, active=1 */

/* Guard velocity saturation. */
calculate(-8.0f, 500.0f, 0.060f, 5.0f, 8.0f, 60.0f);
/* predicted=-38, guard=-60, active=1 */
```

Also assert rejection of `NULL`, `NAN`, negative prediction time, a prediction
time above `1 s`, negative soft boundary, negative gain, and a guard maximum
outside `0..500 mm/s`.

- [ ] **Step 2: Compile to verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_predictive_guard.c `
  Control/ball_predictive_guard.c -lm `
  -o tools/host_tests/test_ball_predictive_guard.exe
```

Expected: compilation fails because the header and implementation do not exist.

- [ ] **Step 3: Implement the header and pure calculation**

Implement validation, zero the result before validation, and calculate:

```c
result->predicted_error_mm =
    position_error_mm -
    velocity_mm_s * prediction_time_s;

excess_mm =
    absolute_float(result->predicted_error_mm) -
    soft_boundary_mm;
if (excess_mm <= 0.0f ||
    guard_gain_per_s == 0.0f ||
    guard_velocity_max_mm_s == 0.0f)
{
    return 1U;
}

guard_magnitude_mm_s = clamp_float(
    guard_gain_per_s * excess_mm,
    0.0f,
    guard_velocity_max_mm_s);
result->guard_velocity_mm_s =
    (result->predicted_error_mm < 0.0f)
        ? -guard_magnitude_mm_s
        : guard_magnitude_mm_s;
result->active = 1U;
return 1U;
```

Use these internal validation ceilings:

```c
#define BALL_PREDICTIVE_GUARD_POSITION_LIMIT_MM (150.0f)
#define BALL_PREDICTIVE_GUARD_VELOCITY_LIMIT_MM_S (500.0f)
#define BALL_PREDICTIVE_GUARD_PREDICTION_LIMIT_S (1.0f)
#define BALL_PREDICTIVE_GUARD_BOUNDARY_LIMIT_MM (150.0f)
#define BALL_PREDICTIVE_GUARD_GAIN_LIMIT_PER_S (100.0f)
#define BALL_PREDICTIVE_GUARD_OUTPUT_LIMIT_MM_S (500.0f)
```

- [ ] **Step 4: Compile and verify GREEN**

Run the Step 2 compile command and:

```powershell
rtk tools/host_tests/test_ball_predictive_guard.exe
```

Expected: both exit `0` with no warning.

---

### Task 4: Integrate the Guard with the Cascade Controller

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_balance.c`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- Consumes: `ball_predictive_guard_calculate()` from Task 3 and the fused observer velocity from Task 2.
- Produces:

```c
void ball_balance_set_predictive_guard_enabled(uint8_t enabled);

extern volatile float ball_balance_predicted_error_mm;
extern volatile float ball_balance_guard_velocity_mm_s;
extern volatile uint8_t ball_balance_guard_active;
```

- [ ] **Step 1: Write controller RED cases**

After restoring simple gains with:

```c
assert(ball_balance_set_cascade_gains(
    2.0f, 0.0f, 1.0f, 0.0f, 500.0f) == 1U);
ball_balance_set_reference(0.0f, 0.0f);
ball_balance_set_predictive_guard_enabled(1U);
```

Add:

```c
/* +6 mm physical deviation, moving outward at +50 mm/s. */
ball_state_observer.position_mm = 6.0f;
ball_state_observer.velocity_mm_s = 50.0f;
ball_balance_update();
assert_close(ball_balance_predicted_error_mm, -9.0f);
assert_close(ball_balance_guard_velocity_mm_s, -32.0f);
assert_close(ball_balance_position_pid_velocity_mm_s, -12.0f);
assert_close(ball_balance_target_velocity_mm_s, -32.0f);
assert(ball_balance_guard_active == 1U);

/* Returning fast enough: predicted error is inside the soft boundary. */
ball_state_observer.position_mm = 6.0f;
ball_state_observer.velocity_mm_s = -50.0f;
ball_balance_update();
assert_close(ball_balance_predicted_error_mm, -3.0f);
assert_close(ball_balance_guard_velocity_mm_s, 0.0f);
assert_close(ball_balance_target_velocity_mm_s, -12.0f);
assert(ball_balance_guard_active == 0U);

/* PI is already stronger than the guard. */
ball_state_observer.position_mm = 9.0f;
ball_state_observer.velocity_mm_s = -50.0f;
ball_balance_update();
assert_close(ball_balance_predicted_error_mm, -6.0f);
assert_close(ball_balance_guard_velocity_mm_s, -8.0f);
assert_close(ball_balance_target_velocity_mm_s, -18.0f);
assert(ball_balance_guard_active == 0U);

/* Negative-direction symmetry. */
ball_state_observer.position_mm = -6.0f;
ball_state_observer.velocity_mm_s = -50.0f;
ball_balance_update();
assert_close(ball_balance_predicted_error_mm, 9.0f);
assert_close(ball_balance_guard_velocity_mm_s, 32.0f);
assert_close(ball_balance_target_velocity_mm_s, 32.0f);
assert(ball_balance_guard_active == 1U);
```

Add a repeated guard-active case using `Ki=1.0`. After disabling the guard and
moving to `position=4 mm`, `velocity=0`, assert the PI target is exactly
`-8.02 mm/s`: `-8.0` comes from position P and `-0.02` is the first allowed
integral candidate on the current non-guarded tick. A more negative result
means the integral accumulated while the guard overrode the PI.

Finally disable the guard at `position=6 mm`, `velocity=50 mm/s` and verify the
target returns to the unmodified PI value `-12 mm/s`. Invalidate the observer
and verify all guard diagnostics reset to zero.

- [ ] **Step 2: Compile to verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_predictive_guard.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: compilation fails because the enable interface and diagnostics do not exist.

- [ ] **Step 3: Add constants, state, interface, and reset behavior**

In `Control/ball_balance.h` add:

```c
#define BALL_BALANCE_GUARD_PREDICTION_S (0.060f)
#define BALL_BALANCE_GUARD_SOFT_BOUNDARY_MM (5.0f)
#define BALL_BALANCE_GUARD_GAIN_PER_S (8.0f)
#define BALL_BALANCE_GUARD_VELOCITY_MAX_MM_S (60.0f)

void ball_balance_set_predictive_guard_enabled(uint8_t enabled);
```

In `Control/ball_balance.c` add a private enable flag and the three public
diagnostics. Clear diagnostics in `reset_controller_state()`. Disabling the
guard through the new setter must clear diagnostics immediately.

- [ ] **Step 4: Merge guard demand after the existing distance limit**

After the distance-limit block and before calculating `velocity_error_mm_s`,
call:

```c
ball_predictive_guard_result_t guard_result;

if (predictive_guard_enabled != 0U &&
    reference_velocity_mm_s >
        -BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S &&
    reference_velocity_mm_s <
        BALL_BALANCE_MOVING_REFERENCE_MIN_MM_S &&
    ball_predictive_guard_calculate(
        raw_position_error_mm,
        velocity_mm_s,
        BALL_BALANCE_GUARD_PREDICTION_S,
        BALL_BALANCE_GUARD_SOFT_BOUNDARY_MM,
        BALL_BALANCE_GUARD_GAIN_PER_S,
        BALL_BALANCE_GUARD_VELOCITY_MAX_MM_S,
        &guard_result) != 0U)
{
    ball_balance_predicted_error_mm =
        guard_result.predicted_error_mm;
    ball_balance_guard_velocity_mm_s =
        guard_result.guard_velocity_mm_s;

    if (guard_result.active != 0U &&
        ((guard_result.guard_velocity_mm_s > 0.0f &&
          target_velocity_mm_s <
              guard_result.guard_velocity_mm_s) ||
         (guard_result.guard_velocity_mm_s < 0.0f &&
          target_velocity_mm_s >
              guard_result.guard_velocity_mm_s)))
    {
        target_velocity_mm_s =
            guard_result.guard_velocity_mm_s;
        ball_balance_guard_active = 1U;
        integral_candidate_allowed = 0U;
    }
}
```

Initialize diagnostics to zero before the conditional each update. Keep the
existing distance-limit diagnostic independent from `guard_active`.

- [ ] **Step 5: Compile and verify GREEN**

Run the Step 2 compile command and:

```powershell
rtk tools/host_tests/test_ball_balance.exe
```

Expected: both exit `0` with no warning.

---

### Task 5: Wire `BALL HOLD` Enablement and Startup-Gate Diagnostics

**Files:**
- Modify: `Control/control.c`
- Modify: `Control/straight_turn_test.h`
- Modify: `Control/straight_turn_test.c`
- Modify: `tools/host_tests/test_straight_turn_timing.c`
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- Test: `tools/host_tests/test_straight_turn_timing.c`

**Interfaces:**
- Consumes: `ball_hold_lap_controller_enabled()` and `ball_balance_set_predictive_guard_enabled()`.
- Produces: guard enablement only in active `BALL HOLD` control and the diagnostic `StraightTurnStartupMotionDetected`.

- [ ] **Step 1: Write the startup diagnostic RED assertion**

In `test_straight_turn_timing.c`, after the first successful
`StraightTurnTest_Run()` in the startup loop, assert:

```c
assert(StraightTurnStartupMotionDetected == 1U);
```

After `StraightTurnTest_Reset()` assert:

```c
assert(StraightTurnStartupMotionDetected == 0U);
```

- [ ] **Step 2: Compile to verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_straight_turn_timing.c -lm `
  -o tools/host_tests/test_straight_turn_timing.exe
```

Expected: compilation fails because `StraightTurnStartupMotionDetected` does not exist.

- [ ] **Step 3: Publish the existing gate state without changing it**

Add to `straight_turn_test.h/.c`:

```c
extern volatile uint8_t StraightTurnStartupMotionDetected;
```

Set it to zero in reset, stop, fault, and non-startup states. In
`straight_turn_command_straight()` assign the return value from
`startup_motion_gate_update()` to both the local decision and the public
diagnostic. Do not change the `0.02 m/s` threshold or two-tick confirmation.

- [ ] **Step 4: Enable the predictive guard only for active `BALL HOLD`**

In `TIMER_0_INST_IRQHandler()`, after controller enablement and before
`ball_balance_update()`, call:

```c
ball_balance_set_predictive_guard_enabled(
    (Menu_Active == 0U &&
     Run_Mode == RUN_MODE_BALL_HOLD_LAP &&
     ball_hold_lap_controller_enabled() != 0U)
        ? 1U : 0U);
```

This call occurs every `5 ms`, so leaving `BALL HOLD`, capture mode, stop,
fault, menu entry, and static mode all disable and clear the guard.

- [ ] **Step 5: Add the new source to the Keil target**

In the `Control` source group of
`keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`, add:

```xml
<File>
  <FileName>ball_predictive_guard.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\ball_predictive_guard.c</FilePath>
</File>
```

Place it beside `ball_braking.c` and `ball_balance.c`; do not edit generated
SysConfig sources.

- [ ] **Step 6: Compile and verify the focused timing test**

Run the Step 2 compile command and:

```powershell
rtk tools/host_tests/test_straight_turn_timing.exe
```

Expected: both exit `0` with no warning.

---

### Task 6: Full Software Verification and Hardware Handoff

**Files:**
- Verify: all existing host tests
- Verify: `tools/host_tests/test_ball_predictive_guard.c`
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- Verify: all scoped source, test, spec, and plan changes

**Interfaces:**
- Consumes: completed Tasks 1 through 5.
- Produces: fresh host-test, ARMCLANG build, formatting, and clean-index evidence; no hardware-performance claim.

- [ ] **Step 1: Compile all nine host tests**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_vision_protocol.c `
  Control/vision_protocol.c `
  -o tools/host_tests/test_vision_protocol.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_straight_turn_timing.c -lm `
  -o tools/host_tests/test_straight_turn_timing.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_startup_motion_gate.c `
  Control/startup_motion_gate.c `
  -o tools/host_tests/test_startup_motion_gate.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_static_task.c `
  Control/ball_static_task.c `
  -o tools/host_tests/test_ball_static_task.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_position_feedforward.c `
  Control/ball_position_feedforward.c -lm `
  -o tools/host_tests/test_ball_position_feedforward.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_braking.c `
  Control/ball_braking.c -lm `
  -o tools/host_tests/test_ball_braking.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_predictive_guard.c `
  Control/ball_predictive_guard.c -lm `
  -o tools/host_tests/test_ball_predictive_guard.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_predictive_guard.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: all nine compilation commands exit `0` with no warning.

- [ ] **Step 2: Execute all nine host tests**

Run:

```powershell
rtk tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_straight_turn_timing.exe
rtk tools/host_tests/test_startup_motion_gate.exe
rtk tools/host_tests/test_ball_static_task.exe
rtk tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_position_feedforward.exe
rtk tools/host_tests/test_ball_braking.exe
rtk tools/host_tests/test_ball_predictive_guard.exe
rtk tools/host_tests/test_ball_balance.exe
```

Expected: all nine executables exit `0`.

- [ ] **Step 3: Force the Keil rebuild**

Run:

```powershell
rtk proxy D:\Infineon\Keli\Keil_v5\UV4\UV4.exe `
  -r keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t MSPM0G3507_Project

rtk rg -n `
  "compiling ball_predictive_guard.c|compiling ball_balance.c|compiling ball_state_observer.c|0 Error\(s\), 0 Warning\(s\)" `
  keil/Objects/empty_LP_MSPM0G3507_nortos_keil.build_log.htm
```

Expected: all three modified control sources compile and the report contains
`0 Error(s), 0 Warning(s)`.

- [ ] **Step 4: Check formatting, index, and workspace state**

Run:

```powershell
rtk git diff --check -- `
  Control tools/host_tests `
  keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  docs/superpowers/specs/2026-08-01-ball-hold-predictive-guard-design.md `
  docs/superpowers/plans/2026-08-01-ball-hold-predictive-guard.md

rtk git diff --cached --quiet
rtk git status --short
```

Expected: scoped diff check exits `0`, the index is empty, and all changes
remain unstaged and uncommitted.

- [ ] **Step 5: Hand off hardware acceptance**

Run `BALL HOLD` at least five times. Record startup-gate state, camera and
fused velocity, current and predicted error, PI/guard/final target velocity,
feedforward pulse, final servo pulse, startup maximum error, each turn's
maximum error, and any reversal.

Do not claim the `+/-10 mm` hardware requirement is met from host tests alone.
If it still fails, change only one of these per hardware round:

1. camera velocity blend;
2. prediction time;
3. guard gain or maximum;
4. startup gate threshold, only if logs prove it opens late.

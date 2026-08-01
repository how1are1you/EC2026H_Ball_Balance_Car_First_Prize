# Ball Fast Position Braking Implementation Plan

> **Superseded for latched target generation:** Use `docs/superpowers/plans/2026-08-01-ball-pi-baseline-braking.md`. This earlier plan remains as the history of the stopping-distance detector and first latch implementation.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve fast step-target motion while adding measured-speed stopping-distance braking, precise endpoint acceptance, and enough diagnostics to tune overshoot without adding a position D term or a second integrator.

**Architecture:** Add a small pure braking module that computes stopping distance and a bounded soft-braking target. `ball_balance` keeps the existing position PI, velocity P, acceleration damping, spatial feedforward, and vehicle acceleration feedforward; it latches braking for the current fixed target, bases the braking target on measured speed, limits the reverse velocity error, and freezes position integration until the 5 mm/10 mm/s endpoint gate is reached. `ball_static_task` uses symmetric endpoint gates.

**Tech Stack:** C99, MSPM0 DriverLib/ARMCLANG, host-side GCC assertion tests, Keil uVision project.

## Global Constraints

- Do not run `git commit`; all implementation and plan changes remain uncommitted.
- Preserve the user's existing feedforward values and unrelated working-tree changes.
- Closed-loop servo output range is `500..2200 us`, quantized to 5 us.
- Control and observer period remains 5 ms.
- Do not add a position-error D term.
- Do not add an independent actuator-bias integrator.
- Preserve the current alpha-beta observer, spatial feedforward, position PI, velocity P, acceleration damping, vehicle acceleration feedforward, and 200 ms vision timeout.
- Far from the target, retain the current step-reference response.
- A braking phase may only activate while the ball is moving toward the target.

---

## File Structure

- Create `Control/ball_braking.h`: public braking configuration, result type, and pure calculation interface.
- Create `Control/ball_braking.c`: validated stopping-distance and braking-decision calculation without hardware dependencies.
- Create `tools/host_tests/test_ball_braking.c`: unit tests for calculation, direction handling, invalid inputs, and boundary behavior.
- Modify `Control/ball_balance.h`: public default braking constants and read-only diagnostics.
- Modify `Control/ball_balance.c`: integrate the braking phase, share servo control limits, and publish saturation/braking diagnostics.
- Modify `tools/host_tests/test_ball_balance.c`: controller integration tests with the real braking and feedforward modules.
- Create `tools/host_tests/test_ball_static_task.c`: state-machine tests using fake servo and controller boundaries.
- Modify `Control/ball_static_task.c`: symmetric precise endpoint gates.
- Modify `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`: add `ball_braking.c` to the Control group.

---

### Task 1: Pure Stopping-Distance Braking Decision

**Files:**
- Create: `Control/ball_braking.h`
- Create: `Control/ball_braking.c`
- Create: `tools/host_tests/test_ball_braking.c`

**Interfaces:**
- Consumes: position error, estimated velocity, stopping deceleration, effective delay, safety margin, and braking release speed.
- Produces:

```c
typedef struct
{
    float stopping_distance_mm;
    float safe_velocity_mm_s;
    uint8_t moving_toward_target;
    uint8_t braking_required;
} ball_braking_result_t;

uint8_t ball_braking_calculate(
    float position_error_mm,
    float velocity_mm_s,
    float stopping_acceleration_mm_s2,
    float delay_s,
    float margin_mm,
    float release_velocity_mm_s,
    ball_braking_result_t *result);
```

- [ ] **Step 1: Write the failing pure-function test**

Create `tools/host_tests/test_ball_braking.c` with these cases:

```c
#include "ball_braking.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.01f);
}

int main(void)
{
    ball_braking_result_t result;

    assert(ball_braking_calculate(
               100.0f, 0.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 3.0f);
    assert(result.moving_toward_target == 0U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 29.133f);
    assert_close(result.safe_velocity_mm_s, 60.498f);
    assert(result.moving_toward_target == 1U);
    assert(result.braking_required == 1U);

    assert(ball_braking_calculate(
               30.0f, 60.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert_close(result.stopping_distance_mm, 18.6f);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, -80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.moving_toward_target == 0U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               -20.0f, -80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.moving_toward_target == 1U);
    assert(result.braking_required == 1U);

    assert(ball_braking_calculate(
               2.0f, 8.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 1U);
    assert(result.braking_required == 0U);

    assert(ball_braking_calculate(
               20.0f, 80.0f, 0.0f, 0.060f, 3.0f,
               10.0f, &result) == 0U);
    assert(ball_braking_calculate(
               NAN, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, &result) == 0U);
    assert(ball_braking_calculate(
               20.0f, 80.0f, 150.0f, 0.060f, 3.0f,
               10.0f, NULL) == 0U);
    return 0;
}
```

- [ ] **Step 2: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_braking.c `
  Control/ball_braking.c -lm `
  -o tools/host_tests/test_ball_braking.exe
```

Expected: compilation fails because `ball_braking.h/.c` do not exist.

- [ ] **Step 3: Implement the public header**

`Control/ball_braking.h` defines the result type and function signature above. Include `<stdint.h>` and use a normal include guard.

- [ ] **Step 4: Implement the minimal validated calculation**

`Control/ball_braking.c` must:

1. Reject a null result pointer before dereferencing it, then clear every result field before validating the numeric inputs.
2. Reject NaN inputs, non-positive stopping acceleration, negative delay/margin/release speed, stopping acceleration above `3000 mm/s^2`, delay above `1.0 s`, margin above `50 mm`, or release speed above `100 mm/s`.
3. Clamp absolute position error to 150 mm and absolute velocity to 500 mm/s.
4. Compute:

```c
stopping_distance_mm =
    speed_mm_s * speed_mm_s /
        (2.0f * stopping_acceleration_mm_s2) +
    speed_mm_s * delay_s +
    margin_mm;

available_distance_mm =
    absolute_error_mm -
    speed_mm_s * delay_s -
    margin_mm;

safe_velocity_mm_s =
    (available_distance_mm > 0.0f) ?
        sqrtf(2.0f * stopping_acceleration_mm_s2 *
              available_distance_mm) :
        0.0f;
```

5. Set `moving_toward_target` only when `position_error_mm * velocity_mm_s > 0`.
6. Set `braking_required` only when moving toward the target, speed is greater than `release_velocity_mm_s`, and remaining distance is less than or equal to stopping distance.

- [ ] **Step 5: Compile and verify GREEN**

Run the Step 2 command, then:

```powershell
rtk tools/host_tests/test_ball_braking.exe
```

Expected: compile and executable both exit 0 with no warning or assertion.

---

### Task 2: Integrate Active Braking and Diagnostics into `ball_balance`

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_balance.c`
- Modify: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- Consumes: `ball_braking_calculate()` and the existing observer state.
- Produces these diagnostics:

```c
extern volatile float ball_balance_stopping_distance_mm;
extern volatile float ball_balance_safe_velocity_mm_s;
extern volatile float ball_balance_braking_velocity_error_mm_s;
extern volatile float ball_balance_unsaturated_pulse_us;
extern volatile uint8_t ball_balance_braking_active;
extern volatile uint8_t ball_balance_braking_creep_active;
extern volatile uint8_t ball_balance_output_saturated;
```

- Uses these initial constants:

```c
#define BALL_BALANCE_STOPPING_ACCEL_MM_S2 (150.0f)
#define BALL_BALANCE_EFFECTIVE_DELAY_S     (0.060f)
#define BALL_BALANCE_BRAKING_MARGIN_MM     (3.0f)
#define BALL_BALANCE_BRAKE_RELEASE_MM_S    (10.0f)
#define BALL_BALANCE_BRAKING_SPEED_FRACTION (0.5f)
#define BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S (20.0f)
#define BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S (10.0f)
#define BALL_BALANCE_BRAKING_SETTLE_POSITION_MM (5.0f)
```

- [ ] **Step 1: Extend the controller test to establish RED**

Update `tools/host_tests/test_ball_balance.c` so it:

- derives feedforward expectations with `ball_position_feedforward_us()` instead of stale literal calibration values;
- initializes controller gains to position Kp 2.0, Ki 0, velocity Kp 1.0, acceleration Kd 0, and velocity limit 500 mm/s;
- verifies that `error=30 mm, velocity=60 mm/s` leaves braking inactive;
- verifies that `error=20 mm, velocity=80 mm/s` activates braking and commands approximately the `60.498 mm/s` safe velocity, limiting the velocity-loop braking error to `19.502 mm/s`;
- verifies that braking remains latched when the instantaneous entry condition clears, uses at most `10 mm/s` creep outside the target band, and releases inside 5 mm at no more than 10 mm/s;
- verifies that the existing position integral is frozen while braking is latched;
- verifies that the unsaturated output and saturation flag are published;
- sets velocity Kp to 20 and applies observer velocities of `-500 mm/s` and `+500 mm/s` at a zero target to verify clamps of `2200 us` and `500 us`;
- retains invalid-observer neutral behavior.

- [ ] **Step 2: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: compilation fails because the new diagnostics do not exist and active braking is not integrated.

- [ ] **Step 3: Share the confirmed servo control limits**

In `ball_balance.c`, remove the duplicate numeric `BALL_BALANCE_CONTROL_MIN/MAX` definitions. Make pulse quantization use:

```c
SERVO_CONTROL_MIN_PULSE_US
SERVO_CONTROL_MAX_PULSE_US
```

This preserves the user's confirmed `500..2200 us` range through the common servo header.

- [ ] **Step 4: Publish output diagnostics at the actuator boundary**

Before quantization, store the requested pulse in `ball_balance_unsaturated_pulse_us`. Set `ball_balance_output_saturated` when it lies outside the shared control limits. Reset these diagnostics when the controller is initialized, disabled, reset, or invalid.

- [ ] **Step 5: Replace the ineffective braking limiter**

Remove `BALL_BALANCE_BRAKING_ACCEL_MM_S2` and the existing
`sqrt(2 * acceleration * error)` block. After position PI and ordinary velocity limiting:

```c
if (reference_velocity_is_zero)
{
    ball_braking_calculate(
        position_error_mm,
        velocity_mm_s,
        BALL_BALANCE_STOPPING_ACCEL_MM_S2,
        BALL_BALANCE_EFFECTIVE_DELAY_S,
        BALL_BALANCE_BRAKING_MARGIN_MM,
        BALL_BALANCE_BRAKE_RELEASE_MM_S,
        &braking);
}

if (braking_entry_required != 0U)
{
    latch_braking_for_current_fixed_target();
}

if (soft_braking_active != 0U)
{
    speed_excess_mm_s =
        max(abs(velocity_mm_s) - braking.safe_velocity_mm_s, 0.0f);
    braking_velocity_error_mm_s =
        min(speed_excess_mm_s,
            BALL_BALANCE_BRAKING_SPEED_FRACTION *
                abs(velocity_mm_s),
            BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S);

    if (abs(position_error_mm) >
            BALL_BALANCE_BRAKING_SETTLE_POSITION_MM &&
        abs(velocity_mm_s) <=
            BALL_BALANCE_BRAKE_RELEASE_MM_S)
    {
        braking_target_magnitude_mm_s =
            min(abs(target_velocity_mm_s),
                BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S);
    }
    else
    {
        braking_target_magnitude_mm_s =
            max(abs(velocity_mm_s) -
                    braking_velocity_error_mm_s,
                0.0f);
    }

    target_velocity_mm_s =
        sign(position_error_mm) *
        braking_target_magnitude_mm_s;
    freeze_position_integral();
}
```

Publish stopping distance, safe velocity, bounded braking velocity error, latch state, and creep state on every valid update. Release the latch inside 5 mm at no more than 10 mm/s, or when the target changes, reference velocity becomes non-zero, the ball moves away from the target, or controller/vision state resets.

- [ ] **Step 6: Preserve the existing feedback law**

Do not add a position D or new integral. Continue:

```c
velocity_error_mm_s =
    target_velocity_mm_s - velocity_mm_s;
ball_balance_proportional_us =
    ball_balance_velocity_kp * velocity_error_mm_s;
ball_balance_derivative_us =
    -ball_balance_velocity_kd * acceleration_mm_s2;
```

- [ ] **Step 7: Compile and verify GREEN**

Run the Step 2 command, then:

```powershell
rtk tools/host_tests/test_ball_balance.exe
```

Expected: compile and executable exit 0.

---

### Task 3: Make Endpoint Acceptance Precise and Symmetric

**Files:**
- Create: `tools/host_tests/test_ball_static_task.c`
- Modify: `Control/ball_static_task.c`

**Interfaces:**
- Uses the existing globals and public state-machine functions.
- Keeps positive dwell at 100 ms and final negative dwell at 300 ms.
- Uses 5 mm position tolerance and 10 mm/s velocity threshold at both endpoints.

- [ ] **Step 1: Write a host state-machine test**

Create a fake boundary test that defines `tick_ms`, `ball_state_observer`, fake servo functions, and a fake `ball_balance_set_reference()`. Drive `ball_static_task_update()` in 5 ms steps and assert:

1. Valid center position becomes ready after 200 ms.
2. Starting moves to `BALL_STATIC_MOVE_POS`.
3. `position=40 mm, velocity=0` remains `BALL_STATIC_MOVE_POS`.
4. `position=50 mm, velocity=11 mm/s` remains `BALL_STATIC_MOVE_POS`.
5. `position=50 mm, velocity=0` enters `BALL_STATIC_HOLD_POS`.
6. Leaving the 5 mm band clears the dwell and returns to move state.
7. A fresh 100 ms stable dwell switches to `BALL_STATIC_MOVE_NEG`.
8. The negative endpoint requires a continuous 300 ms stable dwell before `BALL_STATIC_DONE`.

- [ ] **Step 2: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_static_task.c `
  Control/ball_static_task.c -lm `
  -o tools/host_tests/test_ball_static_task.exe
```

Expected: executable assertion fails because the current positive gate accepts 40 mm and 11 mm/s.

- [ ] **Step 3: Apply symmetric endpoint constants**

Replace separate positive and negative thresholds with:

```c
#define BALL_STATIC_SETTLE_TOLERANCE_MM (5.0f)
#define BALL_STATIC_SETTLE_VELOCITY_MM_S (10.0f)
```

Use them in both move and hold states without changing the 100 ms positive and 300 ms negative dwell durations.

- [ ] **Step 4: Compile and verify GREEN**

Run the Step 2 compile command, then:

```powershell
rtk tools/host_tests/test_ball_static_task.exe
```

Expected: compile and executable exit 0.

---

### Task 4: Keil Integration and Complete Verification

**Files:**
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
- Verify: all host tests and `keil/MSPM0G3507_Project_build.log`

**Interfaces:**
- Adds `Control/ball_braking.c` to the existing Control source group.

- [ ] **Step 1: Add the source to the Keil project**

Insert next to `ball_balance.c`:

```xml
<File>
  <FileName>ball_braking.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\ball_braking.c</FilePath>
</File>
```

- [ ] **Step 2: Compile all affected host tests from source**

Run the exact Task 1, Task 2, and Task 3 GCC commands, then execute all three generated programs. Also recompile and execute:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_state_observer.exe

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_position_feedforward.c `
  Control/ball_position_feedforward.c -lm `
  -o tools/host_tests/test_ball_position_feedforward.exe
rtk tools/host_tests/test_ball_position_feedforward.exe
```

Expected: every compile and executable exits 0.

- [ ] **Step 3: Build the firmware**

Run:

```powershell
rtk proxy D:\Infineon\Keli\Keil_v5\UV4\UV4.exe `
  -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t MSPM0G3507_Project
```

Expected: exit 0; build log reports zero errors and zero warnings.

- [ ] **Step 4: Verify scope without committing**

Run:

```powershell
rtk git diff --check
rtk git status --short
rtk git diff -- Control Hardware tools/host_tests `
  keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  docs/superpowers/plans/2026-08-01-ball-fast-position-braking.md `
  docs/superpowers/specs/2026-08-01-ball-fast-position-braking-design.md
```

Expected:

- no whitespace errors;
- no commit created during implementation;
- the user's feedforward values remain unchanged;
- generated Keil output changes are reported but not staged;
- production changes are limited to braking, controller integration, endpoint gates, and project registration.

- [ ] **Step 5: Record board-only checks**

The local build cannot prove physical performance. On the board, repeat the static task five times and record total time, positive/negative peak position, maximum estimated speed, braking start position, braking-active duration, unsaturated pulse, saturation flag, and final stable error. Then repeat center and arbitrary-position holding during vehicle startup, straight motion, turns, and braking.

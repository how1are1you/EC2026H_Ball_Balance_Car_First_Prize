# Latched Soft Braking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make 100 mm ball-position moves decelerate monotonically through the final 5 mm without reverse rebound or brake-release reacceleration, while preserving the existing fast far-field position response.

**Architecture:** Keep the existing stopping-distance calculation as the brake-entry detector. Add a pure soft-target calculation to `ball_braking`, then let `ball_balance` latch the braking phase for the current fixed target, freeze the existing position integral while latched, and release only inside the 5 mm/10 mm/s endpoint gate or on a safety/reset condition.

**Tech Stack:** C99, MSPM0 bare-metal firmware, host-side GCC assertion tests, ARMCLANG/Keil uVision.

## Global Constraints

- Do not add a position D term or a second integrator.
- Keep spatial feedforward calibration unchanged: `-50:1295`, `-25:1310`, `0:1375`, `+25:1410`, `+50:1455 us`.
- Keep the actuator control range `500..2200 us` and 5 us quantization.
- Keep `a_stop=150 mm/s^2`, `T_delay=0.060 s`, `d_margin=3 mm`, and `v_release=10 mm/s` for the first hardware test.
- Use `k_speed_fraction=0.5`, `brake_velocity_error_max=20 mm/s`, `braking_creep_velocity=10 mm/s`, and `settle_position=5 mm`.
- Preserve normal position PI behavior outside the latched braking phase.
- The 100 mm move may take at most 0.2 s longer than the current accepted baseline.
- The final 5 mm of a 100 mm move must be monotonic, with no reverse rebound or second high-speed acceleration.
- Do not stage or commit any file.

---

### Task 1: Pure Soft-Braking Target Calculation

**Files:**
- Modify: `Control/ball_braking.h`
- Modify: `Control/ball_braking.c`
- Test: `tools/host_tests/test_ball_braking.c`

**Interfaces:**
- Consumes: position error, measured velocity, ordinary position-PI target velocity, safe velocity, and the four soft-braking limits.
- Produces:

```c
typedef struct
{
    float target_velocity_mm_s;
    float braking_velocity_error_mm_s;
    uint8_t creep_active;
} ball_soft_braking_result_t;

uint8_t ball_braking_calculate_soft_target(
    float position_error_mm,
    float velocity_mm_s,
    float normal_target_velocity_mm_s,
    float safe_velocity_mm_s,
    float speed_fraction,
    float velocity_error_max_mm_s,
    float creep_velocity_mm_s,
    float settle_position_mm,
    float release_velocity_mm_s,
    ball_soft_braking_result_t *result);
```

- [ ] **Step 1: Add failing tests for ordinary soft braking**

Append these independently derived cases to `tools/host_tests/test_ball_braking.c`:

```c
ball_soft_braking_result_t soft_result;

assert(ball_braking_calculate_soft_target(
           20.0f,
           80.0f,
           40.0f,
           60.498f,
           0.5f,
           20.0f,
           10.0f,
           5.0f,
           10.0f,
           &soft_result) == 1U);
assert_close(soft_result.target_velocity_mm_s, 60.498f);
assert_close(
    soft_result.braking_velocity_error_mm_s,
    19.502f);
assert(soft_result.creep_active == 0U);

assert(ball_braking_calculate_soft_target(
           -20.0f,
           -80.0f,
           -40.0f,
           60.498f,
           0.5f,
           20.0f,
           10.0f,
           5.0f,
           10.0f,
           &soft_result) == 1U);
assert_close(soft_result.target_velocity_mm_s, -60.498f);
assert_close(
    soft_result.braking_velocity_error_mm_s,
    19.502f);
assert(soft_result.creep_active == 0U);
```

These tests catch implementations that still subtract the speed excess from the smaller position-PI target or apply the wrong sign on negative moves.

- [ ] **Step 2: Add failing tests for low-speed endpoint behavior**

Add:

```c
assert(ball_braking_calculate_soft_target(
           4.0f,
           20.0f,
           8.0f,
           0.0f,
           0.5f,
           20.0f,
           10.0f,
           5.0f,
           10.0f,
           &soft_result) == 1U);
assert_close(soft_result.target_velocity_mm_s, 10.0f);
assert_close(
    soft_result.braking_velocity_error_mm_s,
    10.0f);
assert(soft_result.creep_active == 0U);

assert(ball_braking_calculate_soft_target(
           10.0f,
           5.0f,
           20.0f,
           30.0f,
           0.5f,
           20.0f,
           10.0f,
           5.0f,
           10.0f,
           &soft_result) == 1U);
assert_close(soft_result.target_velocity_mm_s, 10.0f);
assert_close(
    soft_result.braking_velocity_error_mm_s,
    0.0f);
assert(soft_result.creep_active == 1U);
```

The first case proves that the target does not collapse to zero in the final 4 mm. The second proves that a ball which slows early outside the target band receives only a 10 mm/s creep command.

- [ ] **Step 3: Add failing validation tests**

Add:

```c
assert(ball_braking_calculate_soft_target(
           20.0f, 80.0f, 40.0f, 60.0f,
           0.0f, 20.0f, 10.0f, 5.0f, 10.0f,
           &soft_result) == 0U);
assert(ball_braking_calculate_soft_target(
           20.0f, 80.0f, 40.0f, 60.0f,
           0.5f, 20.0f, 10.0f, 5.0f, 10.0f,
           NULL) == 0U);
```

- [ ] **Step 4: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_braking.c `
  Control/ball_braking.c -lm `
  -o tools/host_tests/test_ball_braking.exe
```

Expected: compilation fails because `ball_soft_braking_result_t` and `ball_braking_calculate_soft_target()` do not exist.

- [ ] **Step 5: Declare the result type and function**

Add the interface from this task's **Interfaces** block to `Control/ball_braking.h`, after `ball_braking_result_t` and before the function declarations.

- [ ] **Step 6: Implement the pure calculation**

Add to `Control/ball_braking.c`:

```c
uint8_t ball_braking_calculate_soft_target(
    float position_error_mm,
    float velocity_mm_s,
    float normal_target_velocity_mm_s,
    float safe_velocity_mm_s,
    float speed_fraction,
    float velocity_error_max_mm_s,
    float creep_velocity_mm_s,
    float settle_position_mm,
    float release_velocity_mm_s,
    ball_soft_braking_result_t *result)
{
    float absolute_error_mm;
    float speed_mm_s;
    float normal_target_magnitude_mm_s;
    float speed_excess_mm_s;
    float braking_velocity_error_mm_s;
    float target_magnitude_mm_s;

    if (result == NULL)
    {
        return 0U;
    }

    result->target_velocity_mm_s = 0.0f;
    result->braking_velocity_error_mm_s = 0.0f;
    result->creep_active = 0U;

    if (finite_float(position_error_mm) == 0U ||
        finite_float(velocity_mm_s) == 0U ||
        finite_float(normal_target_velocity_mm_s) == 0U ||
        finite_float(safe_velocity_mm_s) == 0U ||
        finite_float(speed_fraction) == 0U ||
        finite_float(velocity_error_max_mm_s) == 0U ||
        finite_float(creep_velocity_mm_s) == 0U ||
        finite_float(settle_position_mm) == 0U ||
        finite_float(release_velocity_mm_s) == 0U ||
        safe_velocity_mm_s < 0.0f ||
        speed_fraction <= 0.0f ||
        speed_fraction > 1.0f ||
        velocity_error_max_mm_s <= 0.0f ||
        velocity_error_max_mm_s >
            BALL_BRAKING_VELOCITY_LIMIT_MM_S ||
        creep_velocity_mm_s < 0.0f ||
        creep_velocity_mm_s >
            BALL_BRAKING_RELEASE_LIMIT_MM_S ||
        settle_position_mm < 0.0f ||
        settle_position_mm >
            BALL_BRAKING_MARGIN_LIMIT_MM ||
        release_velocity_mm_s < 0.0f ||
        release_velocity_mm_s >
            BALL_BRAKING_RELEASE_LIMIT_MM_S)
    {
        return 0U;
    }

    absolute_error_mm = absolute_float(position_error_mm);
    speed_mm_s = clamp_float(
        absolute_float(velocity_mm_s),
        0.0f,
        BALL_BRAKING_VELOCITY_LIMIT_MM_S);
    normal_target_magnitude_mm_s = clamp_float(
        absolute_float(normal_target_velocity_mm_s),
        0.0f,
        BALL_BRAKING_VELOCITY_LIMIT_MM_S);

    if (absolute_error_mm > settle_position_mm &&
        speed_mm_s <= release_velocity_mm_s)
    {
        target_magnitude_mm_s =
            (normal_target_magnitude_mm_s <
             creep_velocity_mm_s)
                ? normal_target_magnitude_mm_s
                : creep_velocity_mm_s;
        result->creep_active = 1U;
    }
    else
    {
        speed_excess_mm_s =
            speed_mm_s - safe_velocity_mm_s;
        if (speed_excess_mm_s < 0.0f)
        {
            speed_excess_mm_s = 0.0f;
        }
        braking_velocity_error_mm_s =
            speed_fraction * speed_mm_s;
        if (braking_velocity_error_mm_s >
            velocity_error_max_mm_s)
        {
            braking_velocity_error_mm_s =
                velocity_error_max_mm_s;
        }
        if (braking_velocity_error_mm_s >
            speed_excess_mm_s)
        {
            braking_velocity_error_mm_s =
                speed_excess_mm_s;
        }
        target_magnitude_mm_s =
            speed_mm_s - braking_velocity_error_mm_s;
        result->braking_velocity_error_mm_s =
            braking_velocity_error_mm_s;
    }

    if (position_error_mm > 0.0f)
    {
        result->target_velocity_mm_s =
            target_magnitude_mm_s;
    }
    else if (position_error_mm < 0.0f)
    {
        result->target_velocity_mm_s =
            -target_magnitude_mm_s;
    }
    return 1U;
}
```

- [ ] **Step 7: Compile and verify GREEN**

Run the Step 4 command, then:

```powershell
rtk tools/host_tests/test_ball_braking.exe
```

Expected: compilation and executable both exit 0 with no warning.

---

### Task 2: Latch the Soft-Braking Phase in `ball_balance`

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- Consumes: `ball_braking_calculate()` and `ball_braking_calculate_soft_target()` from Task 1.
- Produces:

```c
#define BALL_BALANCE_BRAKING_SPEED_FRACTION (0.5f)
#define BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S (20.0f)
#define BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S (10.0f)
#define BALL_BALANCE_BRAKING_SETTLE_POSITION_MM (5.0f)

extern volatile float
    ball_balance_braking_velocity_error_mm_s;
extern volatile uint8_t ball_balance_braking_creep_active;
```

- `ball_balance_braking_active` changes semantics from an instantaneous entry decision to the latch state for the current fixed target.

- [ ] **Step 1: Change the integration test to establish RED**

Replace the current `error=20 mm, velocity=80 mm/s` assertions in `tools/host_tests/test_ball_balance.c` with:

```c
ball_state_observer.position_mm = 30.0f;
ball_state_observer.velocity_mm_s = 80.0f;
ball_balance_update();
assert_close(ball_balance_target_velocity_mm_s, 60.498f);
assert_close(ball_balance_stopping_distance_mm, 29.133f);
assert_close(ball_balance_safe_velocity_mm_s, 60.498f);
assert(ball_balance_braking_active == 1U);
assert(ball_balance_braking_creep_active == 0U);
assert_close(
    ball_balance_braking_velocity_error_mm_s,
    19.502f);
assert_close(ball_balance_proportional_us, -19.502f);
assert_close(ball_balance_unsaturated_pulse_us, 1399.498f);
assert(fake_servo_pulse_us == 1400U);
```

Expected values use the real feedforward at `x=30 mm`, which is `1419 us`, and velocity Kp `1 us/(mm/s)`.

- [ ] **Step 2: Add a failing latch-persistence test**

Immediately after the entry case, add:

```c
ball_state_observer.position_mm = 35.0f;
ball_state_observer.velocity_mm_s = 20.0f;
ball_balance_update();
assert(ball_balance_braking_active == 1U);
assert(ball_balance_braking_creep_active == 0U);
assert_close(ball_balance_target_velocity_mm_s, 20.0f);
assert_close(
    ball_balance_braking_velocity_error_mm_s,
    0.0f);
```

At this point the instantaneous stopping-distance condition is false and ordinary position PI would request `30 mm/s`. The test proves that the latch remains active and does not restore acceleration.

- [ ] **Step 3: Add failing creep and release tests**

Add:

```c
ball_state_observer.position_mm = 40.0f;
ball_state_observer.velocity_mm_s = 5.0f;
ball_balance_update();
assert(ball_balance_braking_active == 1U);
assert(ball_balance_braking_creep_active == 1U);
assert_close(ball_balance_target_velocity_mm_s, 10.0f);

ball_state_observer.position_mm = 46.0f;
ball_state_observer.velocity_mm_s = 5.0f;
ball_balance_update();
assert(ball_balance_braking_active == 0U);
assert(ball_balance_braking_creep_active == 0U);
assert_close(ball_balance_target_velocity_mm_s, 8.0f);
```

The final target is ordinary position P: `2 × (50 - 46) = 8 mm/s`.

- [ ] **Step 4: Add a failing integral-freeze test**

Reset the controller through `ball_balance_set_cascade_gains()`, enter braking with position Kp 0 and Ki 1, keep the same braking state for 100 updates, then release:

```c
assert(ball_balance_set_cascade_gains(
           0.0f, 1.0f, 1.0f, 0.0f, 500.0f) == 1U);
ball_state_observer.position_mm = 30.0f;
ball_state_observer.velocity_mm_s = 80.0f;
ball_balance_set_reference(50.0f, 0.0f);
for (tick_ms = 0U; tick_ms < 100U; tick_ms++)
{
    ball_balance_update();
    assert(ball_balance_braking_active == 1U);
}

ball_state_observer.position_mm = 46.0f;
ball_state_observer.velocity_mm_s = 5.0f;
ball_balance_update();
assert(ball_balance_braking_active == 0U);
assert(ball_balance_target_velocity_mm_s < 0.1f);
```

Without the freeze, the integral accumulates roughly 10 mm·s and the last assertion fails.

- [ ] **Step 5: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: compilation fails because the new diagnostics and latch behavior do not exist.

- [ ] **Step 6: Replace the old excess-gain constant**

In `Control/ball_balance.h`, remove:

```c
#define BALL_BALANCE_BRAKING_EXCESS_GAIN (0.25f)
```

Add the four constants and two diagnostics from this task's **Interfaces** block.

- [ ] **Step 7: Add latch state and reset behavior**

In `Control/ball_balance.c`, add:

```c
volatile float ball_balance_braking_velocity_error_mm_s;
volatile uint8_t ball_balance_braking_creep_active;

static uint8_t braking_latched;
static float braking_latched_target_mm;
```

Add:

```c
static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t moving_away_from_target(
    float position_error_mm,
    float velocity_mm_s)
{
    return ((position_error_mm > 0.0f &&
             velocity_mm_s < 0.0f) ||
            (position_error_mm < 0.0f &&
             velocity_mm_s > 0.0f))
               ? 1U
               : 0U;
}

static void reset_braking_latch(void)
{
    braking_latched = 0U;
    braking_latched_target_mm = 0.0f;
    ball_balance_braking_active = 0U;
    ball_balance_braking_creep_active = 0U;
    ball_balance_braking_velocity_error_mm_s = 0.0f;
}
```

Call `reset_braking_latch()` from `reset_controller_state()`. Existing disabled, stale-vision, PID-reset, and controller-enable paths already flow through that reset.

- [ ] **Step 8: Replace instantaneous braking with the latch**

Declare:

```c
ball_soft_braking_result_t soft_braking_result;
```

After the ordinary position-PI target is calculated and after `ball_braking_calculate()` publishes stopping distance and safe velocity:

```c
if (braking_latched != 0U &&
    (target_position_mm != braking_latched_target_mm ||
     moving_away_from_target(
         raw_position_error_mm,
         velocity_mm_s) != 0U ||
     (absolute_float(raw_position_error_mm) <=
          BALL_BALANCE_BRAKING_SETTLE_POSITION_MM &&
      absolute_float(velocity_mm_s) <=
          BALL_BALANCE_BRAKE_RELEASE_MM_S)))
{
    reset_braking_latch();
}

if (braking_latched == 0U &&
    braking_result.braking_required != 0U)
{
    braking_latched = 1U;
    braking_latched_target_mm = target_position_mm;
}

if (braking_latched != 0U &&
    ball_braking_calculate_soft_target(
        raw_position_error_mm,
        velocity_mm_s,
        target_velocity_mm_s,
        braking_result.safe_velocity_mm_s,
        BALL_BALANCE_BRAKING_SPEED_FRACTION,
        BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S,
        BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S,
        BALL_BALANCE_BRAKING_SETTLE_POSITION_MM,
        BALL_BALANCE_BRAKE_RELEASE_MM_S,
        &soft_braking_result) != 0U)
{
    target_velocity_mm_s =
        soft_braking_result.target_velocity_mm_s;
    ball_balance_braking_velocity_error_mm_s =
        soft_braking_result.braking_velocity_error_mm_s;
    ball_balance_braking_creep_active =
        soft_braking_result.creep_active;
    ball_balance_braking_active = 1U;
    integral_candidate_allowed = 0U;
}
```

When the reference velocity is outside the existing fixed-target threshold, call `reset_braking_latch()` and skip both braking calculations.

- [ ] **Step 9: Compile and verify GREEN**

Run the Step 5 command, then:

```powershell
rtk tools/host_tests/test_ball_balance.exe
```

Expected: compilation and executable both exit 0 with no warning.

---

### Task 3: Documentation Sync and Full Verification

**Files:**
- Modify: `docs/superpowers/plans/2026-08-01-ball-fast-position-braking.md`
- Verify: `docs/superpowers/specs/2026-08-01-ball-fast-position-braking-design.md`
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: final constant and diagnostic names from Tasks 1 and 2.
- Produces: one non-contradictory description of latched soft braking and fresh build/test evidence.

- [ ] **Step 1: Remove the superseded formula from the earlier plan**

Replace references to `BALL_BALANCE_BRAKING_EXCESS_GAIN`, subtracting speed excess from the position-PI target, and instantaneous `braking_required` behavior with:

```text
braking_entry_required =
    moving_toward_target
    and abs(v) > 10 mm/s
    and abs(error) <= stopping_distance

on braking_entry_required:
    latch braking for the current fixed target

while braking is latched:
    speed_excess = max(abs(v) - v_safe, 0)
    brake_delta_v = min(
        speed_excess,
        0.5 * abs(v),
        20 mm/s)

    if abs(error) > 5 mm and abs(v) <= 10 mm/s:
        target_speed = min(abs(v_pi_limited), 10 mm/s)
    else:
        target_speed = max(abs(v) - brake_delta_v, 0)

    v_target = sign(error) * target_speed
    freeze the existing position integral

release the latch when:
    abs(error) <= 5 mm and abs(v) <= 10 mm/s
    or the fixed target changes
    or reference velocity becomes non-zero
    or the ball moves away from the target
    or vision/controller state resets
```

Use the exact constant and diagnostic identifiers declared in Task 2.

- [ ] **Step 2: Run all eight host-test compiles**

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

rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_braking.c `
  Control/ball_position_feedforward.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: all eight compilation commands exit 0 with no warning.

- [ ] **Step 3: Execute all eight host tests**

Run:

```powershell
rtk tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_straight_turn_timing.exe
rtk tools/host_tests/test_startup_motion_gate.exe
rtk tools/host_tests/test_ball_static_task.exe
rtk tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_position_feedforward.exe
rtk tools/host_tests/test_ball_braking.exe
rtk tools/host_tests/test_ball_balance.exe
```

Expected: all eight executables exit 0.

- [ ] **Step 4: Force a complete Keil rebuild**

Run:

```powershell
rtk proxy D:\Infineon\Keli\Keil_v5\UV4\UV4.exe `
  -r keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t MSPM0G3507_Project
```

Verify:

```powershell
rtk rg -n `
  "compiling ball_braking.c|compiling ball_balance.c|0 Error\(s\), 0 Warning\(s\)" `
  keil/Objects/empty_LP_MSPM0G3507_nortos_keil.build_log.htm
```

Expected: both modified control sources compile and the report contains `0 Error(s), 0 Warning(s)`.

- [ ] **Step 5: Check source scope and leave Git untouched**

Run:

```powershell
rtk proxy git diff --check -- `
  Control Hardware tools/host_tests `
  docs/superpowers/plans/2026-08-01-ball-fast-position-braking.md `
  docs/superpowers/plans/2026-08-01-ball-latched-soft-braking.md `
  docs/superpowers/specs/2026-08-01-ball-fast-position-braking-design.md `
  keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
rtk proxy git diff --cached --quiet
rtk git status --short
```

Expected: scoped diff check exits 0, cached diff exits 0, and all intended changes remain unstaged.

- [ ] **Step 6: Perform the hardware acceptance test**

Run at least five repetitions each of `0 -> +50 mm` and `-50 -> +50 mm`. Record:

```text
total move time
maximum speed
braking-entry position
ball_balance_safe_velocity_mm_s
ball_balance_target_velocity_mm_s
ball_balance_braking_velocity_error_mm_s
ball_balance_braking_active
ball_balance_braking_creep_active
servo pulse
peak endpoint error
```

Accept only if:

```text
100 mm move time increase <= 0.2 s
final 5 mm is monotonic
no reverse rebound
no second high-speed acceleration
stable endpoint error <= 5 mm
```

If the overall braking force is insufficient, increase `BALL_BALANCE_BRAKING_SPEED_FRACTION` first. If endpoint reversal remains, decrease `BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S` first. Change only one constant per hardware test batch.

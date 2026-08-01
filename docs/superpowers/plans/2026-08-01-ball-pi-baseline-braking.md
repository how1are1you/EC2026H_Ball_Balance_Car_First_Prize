# PI-Baseline Latched Braking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore enough braking for `0 -> +50 mm` and `-50 -> +50 mm` moves while preventing the strong endpoint reversal seen after overshoot.

**Architecture:** Keep the existing stopping-distance entry detector, braking latch, low-speed creep, release gate, and position-integral freeze. Replace only the latched target calculation: use limited position PI as the main braking baseline, subtract a small safe-speed excess term, forbid reacceleration, and bound the maximum reverse velocity error.

**Tech Stack:** C99, MSPM0 bare-metal firmware, host-side GCC assertion tests, ARMCLANG/Keil uVision.

## Global Constraints

- Do not add a position D term or another integrator.
- Keep position feedforward, actuator range `500..2200 us`, and 5 us quantization unchanged.
- Use the final hybrid `a_stop=150 mm/s^2`; keep `T_delay=0.060 s`, `d_margin=3 mm`, `v_release=10 mm/s`, `braking_creep_velocity=25 mm/s`, and `settle_position=5 mm`.
- Use `braking_excess_gain=0.5`, `max_braking_speed_fraction=0.6`, and `brake_velocity_error_max=50 mm/s`.
- Preserve normal position PI outside the latched braking phase.
- Preserve the existing latch entry, reset, creep, release, and integral-freeze behavior.
- Do not stage or commit any file.

Tasks 1 through 4 below are retained as the hardware-tuning history. Their intermediate constants and expectations are superseded by the final hybrid policy in Task 5 and the Global Constraints above.

---

### Task 1: Specify the PI-Baseline Target

**Files:**
- Modify: `tools/host_tests/test_ball_braking.c`
- Test: `tools/host_tests/test_ball_braking.c`

**Interfaces:**
- Consumes: `ball_braking_calculate_soft_target()` with a new `braking_excess_gain` argument before the two maximum-braking limits.
- Produces: a target velocity whose magnitude is based on position PI and whose braking velocity error is `abs(v) - abs(v_target)`.

- [ ] **Step 1: Change the high-speed positive and negative expectations**

For `error=+/-20 mm`, `velocity=+/-80 mm/s`, PI target `+/-40 mm/s`, and `v_safe=60.498 mm/s`, pass `0.25`, `0.6`, and `50.0` as the three braking parameters and expect target magnitude `35.1245 mm/s` and braking velocity error `44.8755 mm/s`.

- [ ] **Step 2: Change the endpoint expectation**

For `error=4 mm`, `velocity=20 mm/s`, PI target `8 mm/s`, and `v_safe=0`, expect target `8 mm/s` and braking velocity error `12 mm/s`.

- [ ] **Step 3: Compile and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_braking.c `
  Control/ball_braking.c -lm `
  -o tools/host_tests/test_ball_braking.exe
rtk tools/host_tests/test_ball_braking.exe
```

Expected: compilation or execution fails because the current implementation still returns `60.498 mm/s` for the high-speed case and `10 mm/s` for the endpoint case.

---

### Task 2: Implement the PI-Baseline Target

**Files:**
- Modify: `Control/ball_braking.h`
- Modify: `Control/ball_braking.c`
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_balance.c`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_braking.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- `ball_braking_calculate_soft_target()` accepts `braking_excess_gain`, `max_braking_speed_fraction`, and `velocity_error_max_mm_s`.
- `BALL_BALANCE_BRAKING_EXCESS_GAIN` is `0.25f`.
- `BALL_BALANCE_BRAKING_MAX_SPEED_FRACTION` is `0.6f`.
- `BALL_BALANCE_BRAKING_VELOCITY_ERROR_MAX_MM_S` is `50.0f`.

- [ ] **Step 1: Extend validation**

Reject non-finite inputs, `braking_excess_gain < 0` or `> 1`, `max_braking_speed_fraction <= 0` or `> 1`, and invalid maximum velocity error using the existing braking limits.

- [ ] **Step 2: Replace the normal-speed branch**

Implement:

```text
speed_excess = max(speed - safe_velocity, 0)
desired_target = max(pi_target - 0.25 * speed_excess, 0)
max_brake_delta = min(0.6 * speed, 50)
minimum_target = max(speed - max_brake_delta, 0)
target_magnitude = clamp(desired_target, minimum_target, speed)
braking_velocity_error = speed - target_magnitude
```

Keep the existing target-band-external low-speed creep branch unchanged.

- [ ] **Step 3: Update the integration expectations**

At `position=30 mm`, target `50 mm`, and velocity `80 mm/s`, expect `target_velocity=35.1245 mm/s`, `braking_velocity_error=44.8755 mm/s`, proportional correction `-44.8755 us`, unsaturated pulse `1374.1245 us`, and quantized pulse `1375 us`.

- [ ] **Step 4: Compile and verify GREEN**

Run both focused host-test compile commands and executables. Expected: both exit 0 with no warnings.

---

### Task 3: Full Verification

**Files:**
- Verify: all eight host tests
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

**Interfaces:**
- Consumes: the final braking interface and constants from Task 2.
- Produces: fresh host-test, source-diff, and firmware-build evidence.

- [ ] **Step 1: Compile and execute all eight host tests**

Use the existing commands in `docs/superpowers/plans/2026-08-01-ball-latched-soft-braking.md`. Expected: all eight compiles and all eight executables exit 0.

- [ ] **Step 2: Force the Keil rebuild**

Run:

```powershell
rtk proxy D:\Infineon\Keli\Keil_v5\UV4\UV4.exe `
  -r keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t MSPM0G3507_Project
```

Expected: `0 Error(s), 0 Warning(s)`.

- [ ] **Step 3: Confirm repository state**

Run `rtk proxy git diff --check`, `rtk proxy git diff --cached --quiet`, and `rtk git status --short`. Expected: no diff errors, an empty index, and all changes left unstaged.

- [ ] **Step 4: Hardware acceptance**

Repeat `0 -> +50 mm` and `-50 -> +50 mm` at least five times. Record peak overshoot and whether the last 5 mm is monotonic. Software verification cannot establish physical overshoot; accept the control change only after these on-board observations improve.

---

### Task 4: Enter Braking Earlier After Hardware Feedback

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- `BALL_BALANCE_STOPPING_ACCEL_MM_S2` changes from `150.0f` to `120.0f`.
- The target generator, latch, creep, release, and integral-freeze interfaces remain unchanged.

- [ ] **Step 1: Establish RED at the new braking-entry boundary**

At position error `32 mm` and measured velocity `80 mm/s`, expect stopping distance `34.467 mm`, safe velocity `76.210 mm/s`, an active braking latch, and target velocity `63.053 mm/s`. With `a_stop=150 mm/s^2`, stopping distance is only `29.133 mm`, so the old implementation remains outside braking and the test fails.

- [ ] **Step 2: Change only the stopping-acceleration estimate**

Set:

```c
#define BALL_BALANCE_STOPPING_ACCEL_MM_S2 (120.0f)
```

- [ ] **Step 3: Verify GREEN and rebuild**

Recompile and execute all eight host tests, then force the Keil rebuild. Expected: all host tests exit 0 and Keil reports `0 Error(s), 0 Warning(s)`.

- [ ] **Step 4: Repeat hardware acceptance**

Run both moves at least five times. Confirm that reverse rebound remains absent while peak overshoot decreases from the current approximately `5 mm` and `10 mm` values.

---

### Task 5: Restore Strong Braking Outside the Endpoint Band

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_braking.c`
- Modify: `tools/host_tests/test_ball_braking.c`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_braking.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- `BALL_BALANCE_STOPPING_ACCEL_MM_S2` returns to `150.0f`.
- `BALL_BALANCE_BRAKING_EXCESS_GAIN` changes to `0.5f`.
- Outside the 5 mm endpoint band, the target has a 20 mm/s continuous-approach floor but no maximum reverse-braking clamp.
- Inside the 5 mm endpoint band, the existing `0.6 * speed` and `50 mm/s` maximum reverse-error limits remain active.

- [ ] **Step 1: Establish RED for restored strong braking**

For `error=20 mm`, `velocity=80 mm/s`, PI target `40 mm/s`, and safe velocity `60.498 mm/s`, expect target `30.249 mm/s` and braking velocity error `49.751 mm/s`. The current all-range force limit raises the target to `32 mm/s`, so this test must fail before implementation.

- [ ] **Step 2: Preserve the endpoint protection**

Keep the existing `error=4 mm`, `velocity=20 mm/s`, PI target `8 mm/s` test expecting target `8 mm/s` and braking velocity error `12 mm/s`.

- [ ] **Step 3: Implement the split target policy**

Outside 5 mm:

```text
aggressive_target = max(pi_target - 0.5 * speed_excess, 0)
creep_floor = min(pi_target, 10)
target = clamp(aggressive_target, creep_floor, max(speed, creep_floor))
```

Inside 5 mm, clamp the same aggressive target between the existing minimum target and actual speed.

- [ ] **Step 4: Restore the original braking-entry estimate**

Set `BALL_BALANCE_STOPPING_ACCEL_MM_S2` to `150.0f`, then restore the integration boundary case at `error=20 mm`, `velocity=80 mm/s`.

- [ ] **Step 5: Run full verification**

Compile and execute all eight host tests, force the Keil rebuild, run the scoped diff check, and confirm the Git index remains empty.

---

### Task 6: Prevent a Full Stop Before Continuous Approach

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- `BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S` changes from `10.0f` to `20.0f`.
- `BALL_BALANCE_BRAKE_RELEASE_MM_S` and the final task arrival threshold remain `10.0f`.
- Strong-braking gain, stopping-distance entry, endpoint protection, latch, and integral freeze remain unchanged.

- [ ] **Step 1: Establish RED**

In the latched state at `error=10 mm` and measured speed `5 mm/s`, expect target velocity `20 mm/s`. The current caller still passes the old `10 mm/s` floor, so the integration test must fail.

- [ ] **Step 2: Change only the approach floor**

Set:

```c
#define BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S (20.0f)
```

- [ ] **Step 3: Verify**

Run both focused braking tests, all eight host tests, the Keil forced rebuild, scoped diff check, and empty-index check.

---

### Task 7: Relax the Continuous-Approach Floor

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `tools/host_tests/test_ball_balance.c`
- Test: `tools/host_tests/test_ball_balance.c`

**Interfaces:**
- `BALL_BALANCE_BRAKING_CREEP_VELOCITY_MM_S` changes from `20.0f` to `25.0f`.
- All strong-braking, endpoint-protection, release, and arrival parameters remain unchanged.

- [ ] **Step 1: Establish RED**

In the latched state at `error=15 mm` and measured speed `20 mm/s`, the position PI target is `30 mm/s`; expect the continuous-approach clamp to return `25 mm/s`, while the current caller returns `20 mm/s`.

- [ ] **Step 2: Change the one constant and verify**

Set the continuous-approach floor to `25.0f`, then run the complete host and Keil verification sequence.

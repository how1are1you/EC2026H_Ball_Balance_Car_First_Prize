# Continuous Distance-Limited Position PI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the position PI in control continuously while using remaining distance only as a stateless velocity limit, eliminating brake-latch stop/restart behavior.

**Architecture:** Replace stopping-distance entry, latch, soft target, creep, and endpoint braking modes with a pure delay-compensated distance-velocity calculation. Apply its magnitude as an upper bound to the position-PI velocity only for fixed targets outside 5 mm; keep the existing velocity loop, feedforward, observer, safety handling, and final-zone PI unchanged.

**Tech Stack:** C99, MSPM0 bare-metal firmware, host-side GCC assertion tests, ARMCLANG/Keil uVision.

## Global Constraints

- Do not change PID gains, observer gains, position feedforward, actuator range, or 5 us quantization in this task.
- Use `a_profile=30 mm/s^2`, `T_delay=0.060 s`, and distance-limit activation only when `abs(error)>5 mm`.
- Do not add position D, another integrator, a time trajectory, a latch, a creep phase, or a minimum approach speed.
- Freeze the position-integral candidate only while the distance velocity limit is actively below the PI velocity magnitude.
- Bypass the distance limit when reference velocity is non-zero.
- Do not stage or commit any file.

---

### Task 1: Pure Distance Velocity Limit

**Files:**
- Modify: `Control/ball_braking.h`
- Modify: `Control/ball_braking.c`
- Modify: `tools/host_tests/test_ball_braking.c`

**Interfaces:**

```c
uint8_t ball_braking_calculate_distance_velocity_limit(
    float position_error_mm,
    float profile_acceleration_mm_s2,
    float delay_s,
    float *velocity_limit_mm_s);
```

- [ ] **Step 1: Write RED tests**

Use literal expectations:

```text
error 0 mm   -> 0.000 mm/s
error 20 mm  -> 32.888 mm/s
error -20 mm -> 32.888 mm/s
error 100 mm -> 75.681 mm/s
```

Reject zero acceleration, negative delay, non-finite input, and null output.

- [ ] **Step 2: Verify RED**

Compile the pure test. Expected: compilation fails because the new interface does not exist.

- [ ] **Step 3: Replace historical braking calculations**

Implement:

```text
aT = profile_acceleration * delay
limit = sqrt(aT^2 + 2 * profile_acceleration * abs(error)) - aT
```

Validate and clamp inputs using the existing finite-value pattern.

- [ ] **Step 4: Verify GREEN**

Compile and execute `test_ball_braking`; expect exit 0 with no warning.

---

### Task 2: Keep Position PI in Control

**Files:**
- Modify: `Control/ball_balance.h`
- Modify: `Control/ball_balance.c`
- Modify: `tools/host_tests/test_ball_balance.c`

**Interfaces:**

```c
#define BALL_BALANCE_DISTANCE_PROFILE_ACCEL_MM_S2 (30.0f)
#define BALL_BALANCE_DISTANCE_LIMIT_MIN_ERROR_MM (5.0f)

extern volatile float
    ball_balance_position_pid_velocity_mm_s;
extern volatile float
    ball_balance_distance_velocity_limit_mm_s;
extern volatile uint8_t ball_balance_distance_limited;
```

- [ ] **Step 1: Write controller RED tests**

Verify:

```text
error=30, v_pid=60 -> v_distance=40.665, v_target=40.665
error=20, v_pid=40 -> v_distance=32.888, v_target=32.888
error=15, v_pid=30 -> v_distance=28.254, v_target=28.254
error=10, v_pid=20 -> v_distance=22.761, v_target=20
error=4,  v_pid=8  -> final-zone v_target=8
```

Also verify negative symmetry, non-zero reference-velocity bypass, integral freeze while limited, stale-vision reset, output saturation, and the absence of latch/creep diagnostics.

- [ ] **Step 2: Verify RED**

Compile the controller test. Expected: compilation fails because new diagnostics do not exist and old latch behavior still owns the target.

- [ ] **Step 3: Replace the braking state machine**

After PI target saturation:

```text
position_pid_velocity = v_pid_limited
distance_limit = calculate(abs(raw_error))

if fixed_target
   and abs(raw_error) > 5
   and abs(v_pid_limited) > distance_limit:
    v_target = sign(v_pid_limited) * distance_limit
    distance_limited = true
    freeze integral candidate
```

Remove latch state, soft-target calls, creep handling, and their reset paths.

- [ ] **Step 4: Verify GREEN**

Compile and execute both focused tests with no warnings.

---

### Task 3: Full Verification

**Files:**
- Verify: all eight host tests
- Verify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`

- [ ] **Step 1: Compile and execute all eight host tests**

Expected: all compilation commands and executables exit 0.

- [ ] **Step 2: Force the Keil rebuild**

Expected: `0 Error(s), 0 Warning(s)`.

- [ ] **Step 3: Check the working tree**

Run the scoped diff check and empty-index check. Leave every change unstaged and uncommitted.

- [ ] **Step 4: Hardware acceptance**

Test `0 -> +50 mm` and `-50 -> +50 mm` at least five times. Record maximum speed, distance limit, PI velocity, final target velocity, pulse, peak overshoot, and whether any complete stop/restart remains.

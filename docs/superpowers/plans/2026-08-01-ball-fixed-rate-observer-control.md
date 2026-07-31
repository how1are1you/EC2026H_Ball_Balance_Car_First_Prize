# Ball Fixed-Rate Observer Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run ball estimation and control every 5 ms, use vision frames only to correct an α-β observer, and replace velocity-error differentiation with `-Kd * estimated_acceleration`.

**Architecture:** Keep UART responsible for validated raw measurements, add a hardware-independent observer that predicts at 200 Hz and corrects on new frames, then make the existing cascade controller consume the observer state every tick. Static/hold tasks and OLED diagnostics read the same observer state so the whole ball subsystem uses one position/velocity source.

**Tech Stack:** C99, TI MSPM0 DriverLib, Keil ARMCLANG/uVision, host-side MinGW GCC tests.

## Global Constraints

- Do not run `git commit`, stage files, push, or create a pull request.
- Do not modify generated `ti_msp_dl_config.c/.h`; the timer is already configured for 5 ms.
- Keep the servo PWM period at 20 ms and retain the existing 10 us effective command step.
- Use `Ts = 0.005 s`, `alpha = 0.55`, `beta = 0.10`, `accel_alpha = 0.20`.
- Limit observer velocity to ±500 mm/s, raw acceleration to ±3000 mm/s², and filtered acceleration to ±2000 mm/s².
- Treat vision data as stale after 200 ms; stale state makes the controller clear dynamic state and command servo neutral.
- Preserve position PI anti-windup, braking-speed limit, vehicle-acceleration feedforward, pulse limits, and all existing task thresholds.
- Add host tests under `tools/host_tests/`; generated `.exe` files remain ignored by the existing executable rule.

---

### Task 1: Extract and Test Raw Vision Payload Decoding

**Files:**
- Create: `Control/vision_protocol.h`
- Create: `Control/vision_protocol.c`
- Create: `tools/host_tests/test_vision_protocol.c`
- Modify: `Control/uart_callback.c:11-135`

**Interfaces:**
- Produces: `uint8_t vision_protocol_decode(const uint8_t payload[8], float *position_mm, float *velocity_mm_s)`.
- Preserves: `vision_ball_position_mm`, `vision_ball_velocity_mm_s`, frame count, timestamp, validity, and UART error globals.

- [ ] **Step 1: Write the failing decoder test**

Create `tools/host_tests/test_vision_protocol.c` with literal IEEE-754 little-endian fixtures:

```c
#include "vision_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

int main(void)
{
    const uint8_t valid_payload[8] = {
        0x00U, 0x00U, 0xA0U, 0x3FU,
        0x00U, 0x00U, 0x20U, 0xC0U
    };
    const uint8_t nan_payload[8] = {
        0x00U, 0x00U, 0xC0U, 0x7FU,
        0x00U, 0x00U, 0x00U, 0x00U
    };
    float position_mm = 0.0f;
    float velocity_mm_s = 0.0f;

    assert(vision_protocol_decode(
               valid_payload,
               &position_mm,
               &velocity_mm_s) == 1U);
    assert(fabsf(position_mm - 12.5f) < 0.001f);
    assert(fabsf(velocity_mm_s + 25.0f) < 0.001f);
    assert(vision_protocol_decode(
               nan_payload,
               &position_mm,
               &velocity_mm_s) == 0U);
    assert(vision_protocol_decode(
               NULL,
               &position_mm,
               &velocity_mm_s) == 0U);
    return 0;
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_vision_protocol.c `
  Control/vision_protocol.c -lm `
  -o tools/host_tests/test_vision_protocol.exe
```

Expected: compilation fails because `vision_protocol.h/.c` do not exist.

- [ ] **Step 3: Implement the minimal decoder**

Define the public function in `vision_protocol.h`. In `vision_protocol.c`, reconstruct both `uint32_t` values in little-endian order, reject null arguments and exponent `0x7F800000`, copy valid bit patterns into floats with `memcpy`, and convert cm/cm/s to mm/mm/s:

```c
uint8_t vision_protocol_decode(
    const uint8_t payload[8],
    float *position_mm,
    float *velocity_mm_s)
{
    uint32_t raw_position;
    uint32_t raw_velocity;
    float position_cm;
    float velocity_cm_s;

    if (payload == NULL ||
        position_mm == NULL ||
        velocity_mm_s == NULL)
    {
        return 0U;
    }

    raw_position = read_u32_le(payload);
    raw_velocity = read_u32_le(payload + 4U);
    if ((raw_position & 0x7F800000UL) == 0x7F800000UL ||
        (raw_velocity & 0x7F800000UL) == 0x7F800000UL)
    {
        return 0U;
    }

    memcpy(&position_cm, &raw_position, sizeof(position_cm));
    memcpy(&velocity_cm_s, &raw_velocity, sizeof(velocity_cm_s));
    *position_mm = position_cm * 10.0f;
    *velocity_mm_s = velocity_cm_s * 10.0f;
    return 1U;
}
```

- [ ] **Step 4: Verify the decoder test is GREEN**

Run the Step 2 compile command, then:

```powershell
rtk tools/host_tests/test_vision_protocol.exe
```

Expected: both commands exit 0 with no assertion failure.

- [ ] **Step 5: Publish decoded values without UART-layer filters**

In `uart_callback.c`:

- include `vision_protocol.h`;
- delete `VISION_POSITION_FILTER_ALPHA`, `VISION_VELOCITY_FILTER_ALPHA`, filter timing constants, and all `vision_filter_*` state;
- replace the body of `vision_uart_publish_sample()` with a call to `vision_protocol_decode()`;
- on failure increment `vision_uart_error_count` without publishing;
- on success assign raw converted position and velocity, then timestamp, validity, and finally increment frame count;
- remove obsolete filter resets from `vision_uart_reset()`.

- [ ] **Step 6: Re-run the decoder test and check the diff**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_vision_protocol.c `
  Control/vision_protocol.c -lm `
  -o tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_vision_protocol.exe
rtk git diff --check
```

Expected: test exits 0 and `git diff --check` prints nothing.

---

### Task 2: Implement the 5 ms α-β State Observer

**Files:**
- Create: `Control/ball_state_observer.h`
- Create: `Control/ball_state_observer.c`
- Create: `tools/host_tests/test_ball_state_observer.c`

**Interfaces:**
- Consumes: a coherent `ball_vision_measurement_t` snapshot supplied once per timer tick.
- Produces: global `ball_state_observer`, `ball_state_observer_reset()`, and `ball_state_observer_update()`.

Define these types and functions:

```c
typedef struct
{
    float position_mm;
    float velocity_mm_s;
    uint32_t frame_count;
    uint32_t sample_ms;
    uint8_t valid;
} ball_vision_measurement_t;

typedef struct
{
    volatile float position_mm;
    volatile float velocity_mm_s;
    volatile float acceleration_mm_s2;
    volatile uint32_t last_frame_count;
    volatile uint32_t last_measurement_ms;
    volatile uint32_t update_count;
    volatile uint8_t initialized;
    volatile uint8_t valid;
    float previous_velocity_mm_s;
} ball_state_observer_t;

extern ball_state_observer_t ball_state_observer;

void ball_state_observer_reset(ball_state_observer_t *observer);
void ball_state_observer_update(
    ball_state_observer_t *observer,
    const ball_vision_measurement_t *measurement,
    uint32_t now_ms);
```

- [ ] **Step 1: Write failing observer behavior tests**

Create `tools/host_tests/test_ball_state_observer.c` with three independent cases:

```c
#include "ball_state_observer.h"

#include <assert.h>
#include <math.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    ball_state_observer_t observer;
    ball_vision_measurement_t measurement = {
        0.0f, 100.0f, 1U, 0U, 1U
    };

    ball_state_observer_reset(&observer);
    assert(observer.valid == 0U);

    ball_state_observer_update(&observer, &measurement, 0U);
    assert(observer.valid == 1U);
    assert_close(observer.position_mm, 0.0f);
    assert_close(observer.velocity_mm_s, 100.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);

    ball_state_observer_update(&observer, &measurement, 5U);
    assert_close(observer.position_mm, 0.5f);
    assert_close(observer.velocity_mm_s, 100.0f);

    ball_state_observer_update(&observer, &measurement, 10U);
    ball_state_observer_update(&observer, &measurement, 15U);
    measurement.position_mm = 4.0f;
    measurement.frame_count = 2U;
    measurement.sample_ms = 20U;
    ball_state_observer_update(&observer, &measurement, 20U);
    assert_close(observer.position_mm, 3.1f);
    assert_close(observer.velocity_mm_s, 110.0f);
    assert_close(observer.acceleration_mm_s2, 400.0f);

    ball_state_observer_update(&observer, &measurement, 25U);
    assert_close(observer.position_mm, 3.65f);
    assert_close(observer.velocity_mm_s, 110.0f);
    assert_close(observer.acceleration_mm_s2, 320.0f);

    ball_state_observer_update(&observer, &measurement, 221U);
    assert(observer.valid == 0U);
    assert_close(observer.velocity_mm_s, 0.0f);
    assert_close(observer.acceleration_mm_s2, 0.0f);
    return 0;
}
```

This catches missing 5 ms prediction, duplicate-frame correction, incorrect `Tvision`, unfiltered acceleration, and missing stale invalidation.

- [ ] **Step 2: Run the observer test and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe
```

Expected: compilation fails because the observer files do not exist.

- [ ] **Step 3: Implement reset, prediction, correction, filtering, and staleness**

Use these constants in `ball_state_observer.h`:

```c
#define BALL_OBSERVER_DT_S                  (0.005f)
#define BALL_OBSERVER_ALPHA                 (0.55f)
#define BALL_OBSERVER_BETA                  (0.10f)
#define BALL_OBSERVER_ACCEL_FILTER_ALPHA    (0.20f)
#define BALL_OBSERVER_VELOCITY_LIMIT_MM_S   (500.0f)
#define BALL_OBSERVER_RAW_ACCEL_LIMIT_MM_S2 (3000.0f)
#define BALL_OBSERVER_ACCEL_LIMIT_MM_S2     (2000.0f)
#define BALL_OBSERVER_VISION_TIMEOUT_MS     (200UL)
#define BALL_OBSERVER_MIN_FRAME_DT_MS       (5UL)
#define BALL_OBSERVER_MAX_FRAME_DT_MS       (200UL)
```

Implement `ball_state_observer_update()` in this order:

1. If initialized, predict position by `velocity * 0.005`.
2. Accept a correction only when the measurement is valid, fresh, and has a different frame count.
3. On the first fresh frame, initialize position from measurement and velocity from the clamped visual velocity.
4. On later frames, clamp `Tvision`, compute residual, apply `alpha` to position and `beta / Tvision` to velocity.
5. Compute and clamp `a_raw`, low-pass it using `accel_alpha`, and clamp the filtered acceleration.
6. If measurement age exceeds 200 ms, clear initialized/valid, velocity, acceleration, and previous velocity while preserving the consumed frame count.
7. Increment `update_count` exactly once per call.

- [ ] **Step 4: Verify observer tests are GREEN**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_state_observer.exe
```

Expected: compile and executable both exit 0.

- [ ] **Step 5: Add boundary tests and keep them GREEN**

Extend the test to verify:

- `NULL` observer or measurement does not crash;
- initialization clamps ±700 mm/s to ±500 mm/s;
- a correction cannot exceed the acceleration limits;
- a stale old frame is not re-consumed after invalidation.

Use literal expected limits from the public header, then rerun the Step 4 commands. Expected: exit 0.

---

### Task 3: Convert the Cascade Controller to Fixed-Rate D-on-Measurement

**Files:**
- Create: `tools/host_tests/test_ball_balance.c`
- Modify: `Control/ball_balance.c:8-450`
- Modify: `Control/ball_balance.h:7-62`

**Interfaces:**
- Consumes: `ball_state_observer.position_mm`, `.velocity_mm_s`, `.acceleration_mm_s2`, and `.valid`.
- Produces: existing control outputs plus `ball_balance_estimated_position_mm`, `ball_balance_estimated_acceleration_mm_s2`, `ball_balance_proportional_us`, and `ball_balance_derivative_us`.
- Preserves: all public gain setters and reference APIs.

- [ ] **Step 1: Write failing controller tests**

Create `tools/host_tests/test_ball_balance.c` with real `ball_balance.c`, the real observer global, and a fake servo boundary:

```c
#include "ball_balance.h"
#include "ball_state_observer.h"
#include "servo.h"

#include <assert.h>
#include <math.h>

static uint16_t fake_servo_pulse_us;

void servo_init(void) {}
void servo_set_pulse_us(uint16_t pulse_us)
{
    fake_servo_pulse_us = pulse_us;
}
uint16_t servo_get_pulse_us(void)
{
    return fake_servo_pulse_us;
}

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    ball_state_observer_reset(&ball_state_observer);
    ball_state_observer.valid = 1U;
    ball_state_observer.initialized = 1U;
    ball_state_observer.position_mm = 0.0f;
    ball_state_observer.velocity_mm_s = 10.0f;
    ball_state_observer.acceleration_mm_s2 = 20.0f;

    ball_balance_init();
    assert(ball_balance_set_cascade_gains(
               0.0f, 0.0f, 1.0f, 1.0f, 500.0f) == 1U);
    ball_balance_set_enabled(1U);
    ball_balance_set_reference(0.0f, 0.0f);
    ball_balance_update();

    assert_close(ball_balance_proportional_us, -10.0f);
    assert_close(ball_balance_derivative_us, -20.0f);
    assert(fake_servo_pulse_us == 1220U);

    ball_state_observer.velocity_mm_s = 0.0f;
    ball_state_observer.acceleration_mm_s2 = 0.0f;
    ball_balance_set_reference(50.0f, 0.0f);
    ball_balance_update();
    assert_close(ball_balance_derivative_us, 0.0f);

    ball_state_observer.valid = 0U;
    ball_balance_update();
    assert(fake_servo_pulse_us == SERVO_NEUTRAL_PULSE_US);
    return 0;
}
```

This test catches the old `d(error)/dt`, frame-gated updates, and missing invalid-state neutral output.

- [ ] **Step 2: Run the controller test and verify RED**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
```

Expected: compilation fails because the new diagnostic outputs and observer integration are absent.

- [ ] **Step 3: Implement fixed-rate observer-driven control**

In `ball_balance.c`:

- replace `#include "uart_callback.h"` with `#include "ball_state_observer.h"`;
- remove `BALL_BALANCE_MIN_DT_MS`, `BALL_BALANCE_MAX_DT_MS`, velocity filter alpha, previous frame/sample/error state, and all variable-`dt` logic;
- set `dt_s` to `BALL_OBSERVER_DT_S`;
- if observer state is invalid, set waiting/stale status as appropriate, reset controller dynamic state, and command neutral;
- otherwise read observer position, velocity, and acceleration once at the top of the update;
- run the existing position PI, anti-windup, braking limit, and velocity P calculations every invocation;
- compute:

```c
ball_balance_proportional_us =
    ball_balance_velocity_kp * velocity_error_mm_s;
ball_balance_derivative_us =
    -ball_balance_velocity_kd * acceleration_mm_s2;
correction_unsaturated_us =
    ball_balance_proportional_us +
    ball_balance_derivative_us +
    acceleration_feedforward_us;
```

- publish estimated position, measured velocity, and estimated acceleration diagnostics;
- reset all diagnostic control terms when disabled or invalid.

- [ ] **Step 4: Verify the controller test is GREEN**

Run the Step 2 compile command, then:

```powershell
rtk tools/host_tests/test_ball_balance.exe
```

Expected: both commands exit 0.

- [ ] **Step 5: Run all host tests**

Run:

```powershell
rtk tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_balance.exe
rtk git diff --check
```

Expected: all executables exit 0 and whitespace check is clean.

---

### Task 4: Wire the Observer into the 5 ms Scheduler and Ball Consumers

**Files:**
- Modify: `empty.c:32-84`
- Modify: `Control/control.c:20-151`
- Modify: `Control/ball_static_task.c:1-278`
- Modify: `Control/ball_hold_lap.c:1-220`
- Modify: `Control/show.c:20-29,607-620,744-781`

**Interfaces:**
- Consumes: raw UART globals and `ball_state_observer_update()`.
- Produces: one observer update before task and controller updates on every timer interrupt.

- [ ] **Step 1: Initialize the observer**

Include `ball_state_observer.h` in `empty.c` and call:

```c
vision_uart_reset();
ball_state_observer_reset(&ball_state_observer);
control_uart_reset();
```

Keep this before enabling UART and timer interrupts.

- [ ] **Step 2: Snapshot vision and update the observer before task logic**

Add a focused static helper in `control.c`:

```c
static void update_ball_state_observer(uint32_t now_ms)
{
    ball_vision_measurement_t measurement;
    uint32_t frame_before;
    uint32_t frame_after;

    do
    {
        frame_before = vision_ball_frame_count;
        measurement.position_mm = vision_ball_position_mm;
        measurement.velocity_mm_s = vision_ball_velocity_mm_s;
        measurement.sample_ms = vision_ball_last_update_ms;
        measurement.valid = vision_ball_position_valid;
        frame_after = vision_ball_frame_count;
    } while (frame_before != frame_after);

    measurement.frame_count = frame_after;
    ball_state_observer_update(
        &ball_state_observer,
        &measurement,
        now_ms);
}
```

Call it immediately after `tick_ms += 5U` and sensor acquisition, before `ball_hold_lap_update()`, `ball_static_task_update()`, and `ball_balance_update()`.

- [ ] **Step 3: Make static-task decisions use observer state**

In `ball_static_task.c`:

- include `ball_state_observer.h`;
- make `vision_is_fresh()` return `ball_state_observer.valid`;
- initialize `positive_peak_mm` from observer position;
- use observer position and velocity in `ball_static_task_update()` and `ball_static_task_service()`;
- retain existing 200 ms fault behavior and all target/tolerance values.

- [ ] **Step 4: Make hold-lap state use observer position**

In `ball_hold_lap.c`:

- include `ball_state_observer.h`;
- replace `ball_hold_lap_read_vision()` with a state reader that returns observer position plus `last_frame_count` and `last_measurement_ms`;
- keep capture-window pushes gated by a new frame count;
- update running error/current-position values from observer position while valid;
- keep frame count for scoring/capture bookkeeping and retain all current fault thresholds.

- [ ] **Step 5: Show the observer estimate in ball-mode OLED pages**

In `show.c`, include `ball_state_observer.h`. For `BALL HOLD` ready and `STATIC HYB` pages, use observer validity, position, and velocity instead of raw UART values. Leave generic vision diagnostics unchanged so raw measurement remains inspectable elsewhere.

- [ ] **Step 6: Re-run host tests and inspect all raw-state consumers**

Run:

```powershell
rtk tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_balance.exe
rtk rg -n -e "vision_ball_position_mm" -e "vision_ball_velocity_mm_s" Control
```

Expected: tests exit 0. Remaining raw vision uses are limited to UART publication, the timer snapshot helper, and intentionally retained generic vision diagnostics.

---

### Task 5: Add New Sources to Keil and Verify the Complete Firmware

**Files:**
- Modify: `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx:627-637`
- Verify: `keil/MSPM0G3507_Project_build.log`

**Interfaces:**
- Adds `vision_protocol.c` and `ball_state_observer.c` to the existing `Control` source group.

- [ ] **Step 1: Add both production sources to the Keil project**

Insert these entries in the `Control` group near `uart_callback.c` and `ball_balance.c`:

```xml
<File>
  <FileName>vision_protocol.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\vision_protocol.c</FilePath>
</File>
<File>
  <FileName>ball_state_observer.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\Control\ball_state_observer.c</FilePath>
</File>
```

- [ ] **Step 2: Run all host tests from fresh compiles**

Run:

```powershell
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_vision_protocol.c `
  Control/vision_protocol.c -lm `
  -o tools/host_tests/test_vision_protocol.exe
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl `
  tools/host_tests/test_ball_state_observer.c `
  Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_state_observer.exe
rtk gcc -std=c99 -Wall -Wextra -Werror -IControl -IHardware `
  tools/host_tests/test_ball_balance.c `
  Control/ball_balance.c Control/ball_state_observer.c -lm `
  -o tools/host_tests/test_ball_balance.exe
rtk tools/host_tests/test_vision_protocol.exe
rtk tools/host_tests/test_ball_state_observer.exe
rtk tools/host_tests/test_ball_balance.exe
```

Expected: all six commands exit 0 with no warnings or assertions.

- [ ] **Step 3: Build the Keil target**

Run:

```powershell
rtk proxy D:\Infineon\Keli\Keil_v5\UV4\UV4.exe `
  -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx `
  -t MSPM0G3507_Project
```

Expected: process exits 0 and the build log reports zero errors and zero warnings.

- [ ] **Step 4: Verify source scope and generated-file discipline**

Run:

```powershell
rtk git diff --check
rtk git status --short
rtk git diff -- Control empty.c `
  keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

Expected:

- no whitespace errors;
- only the planned production, host-test, project, spec, and plan files are new or modified;
- generated files under `keil/Objects/` are not intentionally added to source changes;
- no Git commit, staging, push, or PR action has occurred.

- [ ] **Step 5: Record required board verification**

On the target board, record:

1. observer update count grows by 200 per second;
2. between camera frames, estimated position advances by `velocity * 5 ms`;
3. a static target change does not create a D-term kick while estimated acceleration is zero;
4. `BALL_STATIC` completes both endpoint stability conditions;
5. `BALL_HOLD` remains bounded through vehicle start, cruise, and braking;
6. loss of vision for more than 200 ms returns the servo to neutral.

Hardware observations are reported separately because the local build cannot prove physical ball behavior.

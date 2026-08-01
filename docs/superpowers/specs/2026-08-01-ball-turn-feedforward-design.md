# BALL HOLD Turn Feedforward Design

## Goal

Reduce the positive ball-position peak at the entrance of both arcs and the
persistent positive offset through each arc, without changing the vehicle
wheel-speed commands or adding IMU feedback to the ball controller.

The hardware acceptance target remains:

- absolute ball-position error no greater than `10 mm`;
- no new negative overshoot or second reversal at arc entry or exit;
- no behavior change in straight running or `BALL STATIC`.

## Confirmed Behavior

- `ARC_1` and `ARC_2` both produce a positive ball-position error.
- The error reaches its largest value immediately after arc entry.
- A smaller positive offset persists through the rest of each arc.
- The current predictive guard improves the response after the camera detects
  motion, but it cannot act before the first visual position/velocity change.

The disturbance is therefore deterministic in direction and synchronized with
the known vehicle turn command. It has two components:

1. an entry transient caused by the commanded yaw-rate step;
2. a steady component caused by the sustained turn.

## Constraints

- Do not modify the left/right wheel targets, turn radius, vehicle speed,
  braking profile, or arc-transition geometry.
- Do not use measured yaw rate, lateral acceleration, or any new IMU signal in
  the ball controller.
- Keep the current camera-velocity fusion and predictive guard.
- Keep the current position PI, velocity loop, position feedforward,
  `500..2200 us` servo range, and `5 us` output quantization.
- Do not add an adaptive bias integrator.
- Apply the new compensation only while active `BALL HOLD` owns the ball
  controller.
- Leave all files unstaged and uncommitted.

## Selected Approach

Add a command-based servo feedforward for the ball controller.

The feedforward uses only:

- the known `STRAIGHT_TURN_ARC_1` / `STRAIGHT_TURN_ARC_2` state;
- commanded vehicle speed;
- commanded yaw rate derived by the existing turn controller.

The product

```text
lateral_command = commanded_speed * commanded_yaw_rate
```

is a command-space scaling value. It is not treated as measured or true
centripetal acceleration. It only keeps the feedforward approximately
consistent if the configured speed or radius changes.

For the current nominal settings:

```text
speed  = 0.25 m/s
radius = 0.471 m
|lateral_command| = speed^2 / radius ~= 0.1327 m/s^2
```

Both arcs are clockwise and both observed errors are positive, so the initial
ball-servo correction is negative.

## Feedforward Profile

The feedforward contains a steady hold term and an entry-only extra term:

```text
scale = clamp(
    |lateral_command| / nominal_lateral_command,
    0,
    2)

hold_correction_us =
    -TURN_HOLD_FF_US * scale

entry_correction_us =
    -TURN_ENTRY_EXTRA_US * scale * entry_decay

turn_feedforward_us =
    clamp(
        hold_correction_us + entry_correction_us,
        -TURN_FF_LIMIT_US,
        TURN_FF_LIMIT_US)
```

`entry_decay` starts at `1.0` on the straight-to-arc transition and decreases
linearly to zero over `150 ms`. The hold term remains while the vehicle is in
either arc.

When the vehicle exits an arc, the remaining feedforward is released to zero
over `100 ms`. The release prevents a correction step from creating a negative
ball-position excursion.

Initial hardware values:

```text
TURN_HOLD_FF_US        = 30 us
TURN_ENTRY_EXTRA_US   = 20 us
TURN_ENTRY_DURATION   = 150 ms
TURN_RELEASE_DURATION = 100 ms
TURN_FF_LIMIT_US      = 100 us
```

Only `TURN_HOLD_FF_US` and `TURN_ENTRY_EXTRA_US` are primary tuning
parameters. Durations and the safety limit remain fixed during the first
hardware rounds.

## Timing

`ball_balance_update()` currently runs before `StraightTurnTest_Run()` in each
`5 ms` timer interrupt.

When `StraightTurnTest_Run()` changes the state from straight to `ARC_1` or
`ARC_2`, the wheel differential is not applied until the following control
tick. The turn controller shall publish the upcoming commanded yaw rate at
the state transition. On the next tick:

1. the ball controller sees the arc state and applies the feedforward;
2. `ball_balance_update()` commands the servo;
3. `StraightTurnTest_Run()` applies the arc wheel targets.

This gives the servo one control tick of command lead without predicting from
IMU data or changing the vehicle path.

## Components and Interfaces

### `Control/straight_turn_test.c/.h`

Publish:

```c
extern volatile float StraightTurnCommandOmegaRadS;
```

Requirements:

- set it to the current straight command in straight states;
- set it to `-speed / radius` when entering and running an arc;
- publish the blended command during the existing arc-exit blend;
- clear it in reset, stop, done, and fault paths;
- do not change any existing motor target.

### `Control/ball_turn_feedforward.c/.h`

Provide an isolated state machine that:

- detects the inactive-to-arc transition;
- generates the `150 ms` entry decay;
- generates the steady hold term;
- releases its output over `100 ms` after arc exit;
- rejects non-finite inputs;
- clamps scaling and output;
- publishes output, entry-active, and active diagnostics.

The module shall not access motors, the camera, the IMU, or the servo.

### `Control/ball_balance.c/.h`

Add a setter for the already-calculated servo correction:

```c
void ball_balance_set_turn_feedforward(float correction_us);
```

The setter clamps the correction to the dedicated turn-feedforward limit.
`ball_balance_update()` adds it to:

```text
velocity P + velocity D + longitudinal acceleration FF
```

before the existing servo saturation and quantization.

While the turn feedforward is nonzero, freeze the position integral candidate.
Do not clear the existing integral; preserving it avoids losing the straight
section equilibrium value.

Publish:

```c
ball_balance_turn_feedforward_us
ball_balance_turn_feedforward_active
```

### `Control/control.c`

Own and update the turn-feedforward state every `5 ms`.

Update the module only while the ball controller context is valid:

```text
Menu_Active == 0
Run_Mode == RUN_MODE_BALL_HOLD_LAP
ball_hold_lap_controller_enabled() != 0
```

Pass a separate `arc_active` input that is true only when
`StraightTurnState` is `ARC_1` or `ARC_2`. After an arc, keep updating the
module with `arc_active=0` until its `100 ms` release reaches zero.

Reset the module and pass zero to `ball_balance` immediately when the ball
controller context is invalid, including menu entry, stop, fault, or leaving
`BALL HOLD`. This explicitly keeps `BALL STATIC`, `BALL LAP`, and `DRIBBLE`
unchanged.

## Interaction with Existing Control

- The current predictive guard remains enabled and may add stronger
  center-seeking target velocity if the model under-compensates.
- The distance-based velocity limit remains unchanged.
- The longitudinal startup feedforward remains motion-gated and unchanged.
- The new term is not an integrator and cannot learn or retain a bias between
  runs.
- Vision invalidation continues to force the existing neutral/safe behavior;
  no feedforward is applied without a valid active ball controller update.
- The final servo output remains protected by the existing `500..2200 us`
  clamp and `5 us` quantization.

## Diagnostics

Expose enough information to distinguish command timing from controller
response:

```text
StraightTurnState
StraightTurnCommandSpeed
StraightTurnCommandOmegaRadS
ball_turn_feedforward_entry_active
ball_balance_turn_feedforward_us
ball_balance_predicted_error_mm
ball_balance_guard_velocity_mm_s
ball_balance_guard_active
ball_balance_servo_pulse_us
ball_hold_lap_error_mm
```

No new high-rate serial print is required in the interrupt. Existing
volatile diagnostics can be sampled through the current debug path.

## Tests

### Host tests

Add pure feedforward tests for:

- inactive straight state gives zero output;
- clockwise arc entry produces negative hold plus entry correction;
- entry correction decays to zero after `150 ms`;
- hold correction remains through the arc;
- arc exit releases to zero over `100 ms`;
- both arcs have identical sign;
- command scaling and output clamps;
- non-finite input resets to safe zero.

Extend controller tests for:

- turn feedforward is added to the existing velocity-loop correction;
- the position integral is frozen while turn feedforward is nonzero;
- predictive guard remains independently active;
- disabling or invalidating the controller clears the diagnostic and output;
- servo saturation and quantization remain unchanged.

Extend straight/turn timing tests for:

- commanded yaw rate is published on arc entry;
- the ball feedforward can become active one tick before the arc wheel
  differential is first updated;
- reset, fault, and stop clear the command.

Run all host tests with:

```text
-std=c99 -Wall -Wextra -Werror
```

and force a Keil ARMCLANG rebuild.

### Hardware tuning

Tune one parameter at a time:

1. Set `TURN_ENTRY_EXTRA_US=0`.
2. Adjust `TURN_HOLD_FF_US` in `5 us` increments until the sustained positive
   arc error is close to zero without producing a sustained negative error.
3. Hold that value fixed.
4. Increase `TURN_ENTRY_EXTRA_US` in `5 us` increments until the entry peak is
   within `10 mm`.
5. Reject a setting if it creates a negative peak, visible reversal, or exit
   rebound.
6. Verify both arcs for at least five complete runs.

Record separate peak and steady errors for `ARC_1` and `ARC_2`; do not tune
against only the run-wide maximum.

## Acceptance Criteria

- `ARC_1` and `ARC_2` entry peaks are each within `+/-10 mm`.
- Sustained arc error is centered close enough that it does not approach the
  `+10 mm` boundary under repeat runs.
- No new negative peak beyond `-10 mm`.
- No obvious second reversal at arc entry or exit.
- Straight-start and straight-running behavior are unchanged.
- `BALL STATIC` behavior is unchanged.
- All host tests and the Keil rebuild pass with no new warnings.

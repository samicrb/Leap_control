# Tuning Guide

Every tunable value lives in `config/demo_config.ini`. Reload means restart
the binary — no hot reload. Keep a copy of any config you like before each
tweak.

## 1. General philosophy

1. **Safety first.** Reduce values if anything feels abrupt.
2. **One axis at a time.** Don't change scaling, dead-zone and smoothing
   simultaneously — you will not know which helped.
3. **Write down the value you changed and why.** Tuning is a demo-day job.

## 2. Threshold walkthrough

### 2.1 Hand posture (open / closed fist)

```
gesture.grab_closed_threshold = 0.85
gesture.grab_open_threshold   = 0.30
```

LeapC `grab_strength` is 0 (fully open) → 1 (fully closed).

- Raise `grab_closed_threshold` if the robot starts moving when the user
  only *relaxes* the hand. Typical safe range: `0.80 .. 0.92`.
- Lower `grab_open_threshold` if the system never leaves "closed" (user
  opens but it stays closed). Typical safe range: `0.20 .. 0.40`.
- Always keep a gap of at least **0.4** between the two to prevent
  chattering.

### 2.2 Hand presence loss

```
gesture.hand_loss_timeout_s = 0.15
gesture.min_confidence      = 0.3
```

- `hand_loss_timeout_s` is how long the system tolerates a tracking blip
  before declaring a hand "lost". Raise to `0.25` in noisy lighting.
- `min_confidence` rejects low-confidence frames. Raise if you see flicker.

### 2.3 Motion scaling

```
motion.position_scale         = 0.8   # mm-robot per mm-hand per tick
motion.orientation_scale      = 0.6   # deg per deg
motion.position_deadzone_mm   = 2.5
motion.orientation_deadzone_deg = 2.0
motion.smoothing_alpha        = 0.25  # 0 = no smoothing, 1 = infinite
```

These directly control "how snappy does the robot feel?"

- **Too sluggish** → raise `position_scale` by steps of 0.1, or lower
  `smoothing_alpha` by 0.05.
- **Too jittery** → raise `smoothing_alpha` by 0.05 and raise the
  dead-zone by 0.5 mm.
- Keep `position_scale ≤ 1.5`: beyond that it's hard to stay inside the
  workspace cube without looking frantic.
- Orientation is much more sensitive. Keep `orientation_scale ≤ 0.8`.

### 2.4 Axis sign flips

```
motion.sign_x  = 1    motion.sign_y = 1   motion.sign_z = 1
motion.sign_rx = 1    motion.sign_ry = 1  motion.sign_rz = 1
```

If moving the right hand up makes the robot go down, flip the
corresponding `sign_*` between `1` and `-1`.

### 2.5 Gripper thresholds

```
gripper.distance_open_mm   = 170.0
gripper.distance_close_mm  = 50.0
gripper.neutral_min_mm     = 80.0
gripper.neutral_max_mm     = 140.0
gripper.facing_dot_max     = -0.5
gripper.cooldown_s         = 0.8
gripper.gesture_hold_s     = 0.25
gripper.open_do_index      = 2     # tool DO channel for OPEN coil
gripper.close_do_index     = 1     # tool DO channel for CLOSE coil
```

The gripper module (`ToolIoGripperController`) is end-effector agnostic:
any pneumatic / electric gripper exposing a pair of OPEN/CLOSE coils on
the tool flange works (Schunk, Robotiq, OnRobot, custom). Match the DO
indices to the cell wiring.

Rules of thumb:
- The **neutral band** must contain reasonable resting distances and not
  overlap the open/close thresholds.
- If the gripper double-triggers, raise `cooldown_s`.
- If users struggle to arm the gesture, relax `facing_dot_max` toward
  `-0.3` (less strict).
- `gesture_hold_s` prevents accidental gripper commands while the user
  is just passing through the two-hand posture.

### 2.6 Workspace envelope

```
workspace.enabled            (bool - master switch)
workspace.x_min / x_max      (mm, robot base X)
workspace.y_min / y_max      (mm, robot base Y)
workspace.z_min / z_max      (mm, robot base Z)
workspace.rx_range / ry_range / rz_range   (deg, +/- from safe pose)
```

Shipped default is `workspace.enabled = false` so first-time bring-up runs
with only the speed/accel caps. Once the cell is characterised:

1. Set `workspace.enabled = true`.
2. Start with a small cube around the safe pose. Walk the cell.
3. Gradually widen X/Y/Z until the motion is visually rich but the arm
   cannot reach anything fragile.

> **Orientation convention.** `workspace.rx_range / ry_range / rz_range`
> are applied as `±range` around `safe_pose.rx / ry / rz`. Per the Doosan
> V3.5 convention those fields are **ZYZ' intrinsic Euler (W, P, R)** —
> NOT rotations around the BASE X/Y/Z axes. So `rx_range` constrains W
> (Z-axis yaw), `ry_range` constrains P (Y'-axis pitch),
> `rz_range` constrains R (Z''-axis roll). Pick ranges with that in mind.

### 2.7 Speed / acceleration caps

```
robot.max_lin_speed   = 120.0  # mm/s
robot.max_ang_speed   =  25.0  # deg/s
robot.max_lin_accel   = 400.0  # mm/s^2
robot.max_ang_accel   = 120.0  # deg/s^2
```

These are **hard** caps applied after the workspace guard. Do not raise
them without a safety review.

## 3. Demo-day calibration procedure

1. `dryrun.robot = true` — verify the console UI responds correctly to
   your hands with no motion.
2. Set `robot.ip` / `robot.port`, start in dry-run, check that
   `moveHome` is commanded (visible in the log).
3. `dryrun.robot = false`. Check safe-pose approach speed is comfortable.
4. Run through every state (`docs/OPERATOR_CHECKLIST.md`).
5. Do the pick-and-place three times back-to-back. Tune gripper and
   position-scale to taste.
6. Lock the config and **check it into git** before the first visitor.

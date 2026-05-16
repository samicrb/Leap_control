# Claude Code — Handoff Summary

Snapshot for a fresh Claude Code session to pick up this project without losing
context. Keep this file up to date when the architecture or constraints change.

---

## 1. Project goal

Build a **demo-grade, operator-friendly, gesture-controlled teleoperation
application** that drives a Doosan industrial robot from an Ultraleap Leap
Motion sensor, on a Windows 11 host PC, for an open-house style event.

- **Robot**: Doosan **M1013** (the project was previously targeted at A0912
  then A0509; the safe pose, default IP and UI banner have all been updated
  to M1013). Controller version: V3.5.
- **Robot API**: **DRFL 1.33.3** from the official
  [`DoosanRobotics/API-DRFL`](https://github.com/DoosanRobotics/API-DRFL)
  GitHub release. **Standard (non-RT) API only.**
- **Sensor stack**: Ultraleap **Gemini 6.2.0**, accessed via **LeapC**.
- **Gripper**: any tool-DO–driven two-state gripper (Schunk / Robotiq /
  OnRobot / custom). DO channel indices are configurable.
- **End user**: operator briefs visitor, visitor controls the robot with
  bounded gestures inside a small workspace. Safety-first; conservative
  speeds.

The implementation priority order is:
1. Safety and predictability
2. Event-day robustness
3. Clear operator experience
4. Perceived responsiveness
5. Visual appeal

---

## 2. Current branch and commit

- Branch: **`codex/add-configurable-blending-for-micro-movements`**
- HEAD commit: **`2ae2e53`** — *Velocity-smoothing layer on top of pursuit
  (continuous v profile)*
- Repository: `samicrb/Leap_control`

### Recent history (top → bottom = newest → older)

```
2ae2e53  Velocity-smoothing layer on top of pursuit (continuous v profile)
4399554  Pursuit/lookahead controller + configurable amovel blending
2126605  Fix DRFL amovel signature and blend-type gating
cb8361d  Add configurable micro-motion blending with safer chaining defaults
97883c1  Fix amovel signature + retarget to M1013 @ 192.168.1.25
3c5bb8a  Replace continuous speedl streaming with discrete amovel micro-motion supervisor
47bcff0  Tighten decel-ramp guards: extend UI pose gate + align Config defaults
bc91fbb  Merge pull request #8 (decel-ramp variant)
56a4005  Add non-blocking SpeedL deceleration ramp on active->passive transitions
7bbf3cf  Forbid getCurrentPose during active streaming (alarm 5.7056 root cause)
afdef1f  Transition-driven motion + deadband + no-blending home (alarm 5.7056 mitigation)
d5f4f32  Remove emergencyStop() from the normal shutdown path
e565a90  Restrict emergencyStop() to critical faults only
ff200d6  Refresh context brief to match shipped implementation
de7a447  Split stopMotion (soft) vs emergencyStop (hard) for joystick-style control
```

### Build status

- CMake build (with HAVE_DRFL=0 / HAVE_LEAPC=0 stub adapters) succeeds.
- 11/11 state-machine smoke tests pass
  (`build/state_machine_smoke_test`).
- On Windows with the real SDKs:
  - `DRFL_SDK_ROOT` points at the cloned `API-DRFL` repo
    (`include/DRFLEx.h` + `library/Windows/64bits/DRFLWin64.lib`).
  - `LEAPC_SDK_ROOT` points at the installed `Ultraleap/LeapSDK`.
  - Runtime DLLs (`DRFLWin64.dll`, `PocoFoundation64.dll`,
    `PocoNet64.dll`) are auto-copied next to the binary.

---

## 3. Files in the repository and their roles

```
Leap_control/
├── CMakeLists.txt            build config + SDK auto-detect + DLL copy step
├── README.md                 quick-start / build instructions
├── doosan_gesture_demo_context_v2.md
│                             original product brief + §26 "Implementation
│                             Status" describing the validated DRFL bring-up
│                             sequence and bring-up lessons
├── config/
│   └── demo_config.ini       single source of runtime tuning knobs
├── docs/
│   ├── OPERATOR_CHECKLIST.md cold-start, smoke-test, shutdown checklist
│   ├── TUNING_GUIDE.md       parameter walkthrough + tuning recipes
│   └── CLAUDE_HANDOFF.md     this file
├── src/
│   ├── main.cpp              entry point: load config, wire adapters, run
│   ├── app/
│   │   ├── Application.{hpp,cpp}      tick loop, micro-motion supervisor,
│   │                                  pursuit + velocity smoothing + ramp
│   ├── config/
│   │   ├── Config.{hpp,cpp}           strongly-typed config snapshot + parser
│   ├── sensor/
│   │   ├── HandFrame.hpp              POD: one tracked Leap frame
│   │   ├── ILeapSource.hpp            sensor abstraction
│   │   └── LeapSource.{hpp,cpp}       LeapC adapter (HAVE_LEAPC compile-guard)
│   ├── gesture/
│   │   ├── GestureTypes.hpp           Vec3 helpers + gesture enums/structs
│   │   └── GestureInterpreter.{hpp,cpp}  smooths Leap frames into a
│   │                                     CommandOutput-friendly report
│   ├── state/
│   │   ├── States.hpp                 DemoState + FaultReason enums
│   │   └── StateMachine.{hpp,cpp}     IDLE / READY / POSITION / ORIENTATION /
│   │                                  RECENTER / GRIPPER / FAULT transitions
│   ├── robot/
│   │   ├── RobotPose.hpp              POD: x y z rx ry rz (ZYZ' Euler)
│   │   ├── IRobotController.hpp       adapter interface: connect, engage,
│   │   │                              moveHome, sendCartesianVelocity,
│   │   │                              sendCartesianMicroMove,
│   │   │                              stopMotion, emergencyStop, getCurrentPose
│   │   ├── DrflRobotController.{hpp,cpp}  real DRFL 1.33.3 adapter
│   │   │                                  (HAVE_DRFL compile-guard)
│   │   └── WorkspaceGuard.{hpp,cpp}   workspace cube + orientation cone clamp,
│   │                                  plus pose-less clampSpeed() used during
│   │                                  active streaming
│   ├── gripper/
│   │   ├── IGripperController.hpp     two-state gripper interface
│   │   └── ToolIoGripperController.{hpp,cpp}
│   │                                  generic vendor-agnostic tool-DO gripper
│   ├── input/
│   │   ├── IExternalButton.hpp        external supervisor button interface
│   │   └── KeyboardButton.{hpp,cpp}   keyboard SPACE / Q / ESC implementation
│   ├── ui/
│   │   └── ConsoleUI.{hpp,cpp}        operator console UI
│   └── util/
│       ├── Logger.{hpp,cpp}           thread-safe logger (file + stderr)
│       └── MathUtils.hpp              clamp, EMA, vector helpers
└── test/
    └── state_machine_smoke_test.cpp   11 headless scenarios validating
                                       the state machine transitions
```

---

## 4. Current architecture

### Threads and rates

- Single thread inside `Application::run()` at **60 Hz**.
- `LeapSource` polls in a background thread internally and exposes
  `pollLatest()` non-blocking.
- DRFL callbacks (`onMonitoringState`, `onMonitoringAccessControl`,
  `onLogAlarm`) run on the DRFL internal thread and store into file-scope
  atomics.

### Tick pipeline (`Application::tick`)

```
1. Poll Leap frame (HandFrame)
2. GestureInterpreter → GestureReport (smoothing, posture, gripper gesture)
3. StateMachine.step(report, button, now) → CommandOutput
4. Robot command pipeline based on state:
     - CRITICAL fault (RobotError / InternalError): emergencyStop() one-shot
     - cur_active OR ramp_to_zero_:
         a. On cur_active && !prev_active:
              ONE getCurrentPose() seed → active_entry_robot_pose_,
              desired_target_pose_, last_commanded_target_pose_
              filtered_velocity_ / last_accel_ reset to zero
         b. Recompute desired_target_pose_:
              if cur_active:  entry + ratio * (cmd.velocity / position_scale)
              if ramp_to_zero_: hold last_commanded_target_pose_
         c. Scheduler (period_floor = max(1/command_rate_hz, min_period_s))
         d. Velocity smoothing pipeline (if micro_velocity_filter_enabled):
              v_des  = clamp(Kp * err, vmax)  with Kp = micro_command_rate_hz
              v      = LPF(v_des, alpha)
              v      = accel_limit(v, prev_v, accel_cap * dt)
              a      = (v - prev_v) / dt
              a      = jerk_limit(a, prev_a, jerk_cap * dt)
              v      = prev_v + a * dt
              if |v| < deadband → v = 0
              next_target = last_commanded + v * dt
              radius_eff  = min(blending_radius, 0.4 * segment_norm)
              robot_.sendCartesianMicroMove(next_target, ..., radius_eff)
              last_commanded ← next_target
         e. Ramp-to-zero completion check:
              if !any_velocity OR elapsed >= micro_stop_ramp_time_s:
                  ramp_to_zero_ = false; clear pursuit state
     - prev_active falling edge:
         start ramp_to_zero_ (if velocity filter enabled);
         otherwise just go quiet, last in-flight amovel completes naturally
     - passive stable: NOTHING is sent to the robot
5. Gripper open/close I/O (driven by cmd.openGripper / cmd.closeGripper)
6. UI refresh:
     getCurrentPose() permitted ONLY when !cur_active AND elapsed > 0.5 s
     UI always shows last_pose_for_ui_ (cached)
```

### DRFL bring-up sequence in `engage()`

The single most important sequence in the codebase, validated against the
official API-DRFL Windows example:

```
1.  open_connection(ip, port)
2.  setup_monitoring_version(1)
3.  set_on_monitoring_state / set_on_monitoring_access_control / set_on_log_alarm
4.  manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)
5.  wait for MONITORING_ACCESS_CONTROL_GRANT (timeout = connect_timeout_s)
6.  AUTO-RESET latched safety stops (if auto_reset_safety):
        SAFE_STOP / SAFE_STOP2  → CONTROL_RESET_SAFET_STOP
        SAFE_OFF  / SAFE_OFF2   → CONTROL_RESET_SAFET_OFF
7.  if state == SAFE_OFF: set_robot_control(CONTROL_SERVO_ON)
8.  wait for STATE_STANDBY
9.  set_robot_mode(ROBOT_MODE_AUTONOMOUS)
10. set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP)
11. change_collision_sensitivity(collision_sensitivity)       (0 = disabled)
12. set_singularity_handling(SINGULARITY_AVOIDANCE)           (0 = AVOID)
```

Mastering loss (state stays in `STATE_RECOVERY`) is **NOT** bypassable from
the API — log a clear pointer to the pendant menu.

### moveHome

Uses `movejx` by default (joint-space interpolation, immune to Cartesian
path singularities). Falls back to `movel` if `home_use_movejx = false`.
**No blending radius. No mwait. Polls `STATE_STANDBY` after the call.**

### Active control: micro-motion supervisor

The active path NEVER calls `speedl`, NEVER calls `stopMotion`, NEVER calls
`mwait`, NEVER calls `getCurrentPose` (apart from the one-shot seed).
The only motion primitive on the active path is `amovel` issued via
`sendCartesianMicroMove`.

`amovel` signature in DRFL 1.33.3:
```cpp
amovel(target[6], vel[2], acc[2],
       time = 0,
       MOVE_MODE = ABSOLUTE,
       MOVE_REFERENCE = BASE,
       BLENDING_SPEED_TYPE = DUPLICATE,
       DR_MV_APP = DR_MV_APP_NONE)
```
**There is NO blending-radius parameter on amovel.** Our adapter maps the
caller's requested `blending_radius_mm` onto the enum:
- `radius > 0` → `BLENDING_SPEED_TYPE_DUPLICATE` (preserves velocity across
  segments → smooth chaining)
- `radius == 0` → `BLENDING_SPEED_TYPE_OVERRIDE` (each segment standalone)

---

## 5. Important constraints (DO NOT VIOLATE)

These were learned the hard way through repeated alarm 5.7056 trips.

1. **Stay on standard DRFL API.** Do NOT migrate to RT control without
   explicit user direction.
2. **NEVER call `mwait()`.** A `mwait` after any `movel`-family call is a
   documented Doosan trigger for alarm 5.7056
   (`OPERATION_SAFETY_FUNCTION_SOS_VIOLATION`).
3. **NEVER call `stopMotion()` / `speedl(0)` / `STOP_TYPE_QUICK`** on the
   normal active control path. `stopMotion()` is reserved for the shutdown
   chain (`disconnect → disengage → stopMotion → close_connection`).
4. **NEVER call `getCurrentPose()`** during active streaming. It maps to
   a non-motion DRFL command (`CONTROL_CHECK_CURRENT_TASK_POSITION`) and
   interleaving it between two `amovel`/`speedl` calls trips 5.7056.
   The single seed read at active entry is allowed. The UI cache is gated
   on `!cur_active` plus a 0.5 s throttle.
5. **`emergencyStop()` is for true critical robot/API faults only**
   (`RobotError`, `InternalError`). Operator-level conditions
   (`SensorDisconnected`, `LeftHandLost`, `RightHandLost`,
   `RightHandPostureInvalid`, `WorkspaceLimit`) MUST NOT trigger it.
6. **No 60 Hz robot commands.** The Leap polling loop runs at 60 Hz but
   the robot scheduler runs at `micro_command_rate_hz` (default 10).
   Sitting in a stable passive state issues NO robot command.
7. **No blended motion followed by a non-motion DRFL call.** When blending
   is on, the Application MUST guarantee no `getCurrentPose`, `mwait`,
   `stop`, etc. is issued in between two blended `amovel`s.
8. **Doosan V3.5 rotation convention is ZYZ' intrinsic Euler.** The fields
   `rx / ry / rz` on `RobotPose` are stored under those legacy names but
   map directly to Doosan `(W, P, R)`. Pendant `posx` values flow through
   verbatim.
9. **moveHome with `movejx` is the default.** `movel` to a wrist-singular
   safe pose (e.g. Ry near 96°) trips alarm 3205. Setting
   `singularity_handling = 0` (AVOID) helps the controller blend through
   if `movel` is selected.

### Audit invariants (verify with `grep` on any new commit)

```
grep -rn "mwait"                                     src/  →  zero
grep -rn "robot_\.sendCartesianVelocity"             src/app/  →  zero
grep -rn "robot_\.stopMotion"                        src/app/  →  zero
grep -rn "robot_\.emergencyStop"                     src/app/  →  one site
                                                       (Application::tick,
                                                        critical-fault one-shot)
grep -rn "robot_\.getCurrentPose"                    src/app/  →  two sites
                                                       (active-entry seed,
                                                        UI cache gated !cur_active
                                                        AND 0.5 s throttle)
grep -rn "STOP_TYPE_QUICK"                           src/app/  →  comments only
                                                                 (the only real call
                                                                  site is the adapter's
                                                                  emergencyStop())
```

---

## 6. Known bugs and symptoms (and what causes them)

| Alarm / symptom                                          | Root cause                                                                                          | Mitigation in current code                                                                                                  |
|----------------------------------------------------------|-----------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| `5.7056 OPERATION_SAFETY_FUNCTION_SOS_VIOLATION`         | Joint-accel supervisor caught a velocity step / jerk / non-motion command interleaved with motion   | Micro-motion supervisor, pursuit, velocity smoothing, ramp-to-zero tail, blending only between motion commands              |
| `3205 / 3206` "Change singularity region status"         | `movel` Cartesian path grazes a wrist (J5) singularity                                              | `home_use_movejx = true` and `singularity_handling = 0` (AVOID)                                                             |
| `1041` SAFE_OFF latched at engage                        | Previous session left a SAFE_STOP / SAFE_OFF latched                                                | `auto_reset_safety = true` fires `CONTROL_RESET_SAFET_STOP/_OFF` based on initial state                                     |
| "Mastering lost" popup                                   | Firmware-level mastering check (NOT bypassable via DRFL)                                            | Detect `STATE_RECOVERY` and emit a clear pointer to the pendant menu                                                        |
| Servo OFF on demo exit                                   | `STOP_TYPE_QUICK` called on shutdown / disengage / passive transition                               | Shutdown uses `disconnect()` alone; `disengage()` uses soft `stopMotion()`; passive states emit zero commands                |
| Jerky motion / pauses between segments                   | `amovel` runs a full triangular accel/decel profile if segments are too short / not blended         | Pursuit controller (continuous tracking) + velocity smoothing (continuous v profile) + adaptive blending                    |
| `LNK1104 'DRFL.lib'`                                     | The DRFL 1.33.3 lib is named `DRFLWin64.lib` and lives under `library/Windows/64bits/`              | `CMakeLists.txt` uses `find_library` with multiple candidate names and paths                                                |

There are **no actively-known unsolved bugs** at HEAD. The next hardware
test will confirm whether the velocity smoothing layer eliminates the
remaining perceived jerkiness.

---

## 7. Parameters (everything tunable)

All keys live in `config/demo_config.ini`. Defaults shipped today are tuned
for an A-series cobot bring-up; the user has run on M1013 but the safety
profile is conservative enough to be safe on smaller arms.

### Robot link

| Key                                | Default        | Meaning                                                                              |
|------------------------------------|----------------|--------------------------------------------------------------------------------------|
| `robot.ip`                         | `192.168.1.25` | Controller IP                                                                        |
| `robot.port`                       | `12345`        | DRFL TCP port                                                                        |
| `robot.model`                      | `M1013`        | Display only                                                                         |
| `robot.connect_timeout_s`          | `5.0`          | Authority grant + STATE_STANDBY timeout                                              |
| `robot.skip_move_home`             | `false`        | Bypass the home `movejx` at startup                                                  |
| `robot.collision_sensitivity`      | `0`            | 0 = disabled, 50–75 in production                                                    |
| `robot.singularity_handling`       | `0`            | 0 = AVOID, 1 = STOP, 2 = VEL (Doosan SINGULARITY_AVOIDANCE enum)                     |
| `robot.home_use_movejx`            | `true`         | Use joint-space `movejx` for the home approach                                       |
| `robot.auto_reset_safety`          | `true`         | Auto-reset SAFE_STOP / SAFE_OFF at the start of `engage()`                           |

### Safe pose (Doosan ZYZ' Euler)

| Key             | Default | Meaning                                |
|-----------------|---------|----------------------------------------|
| `safe_pose.x`   | `55.0`  | mm, BASE frame                         |
| `safe_pose.y`   | `400.0` | mm                                     |
| `safe_pose.z`   | `375.0` | mm                                     |
| `safe_pose.rx`  | `33.0`  | deg, == W (rotation about BASE Z)      |
| `safe_pose.ry`  | `96.0`  | deg, == P (rotation about Y')          |
| `safe_pose.rz`  | `110.0` | deg, == R (rotation about Z'')         |

### Control loop / hard caps

| Key                     | Default | Meaning                                |
|-------------------------|---------|----------------------------------------|
| `loop.rate_hz`          | `60`    | Leap / state-machine tick rate         |
| `robot.max_lin_speed`   | `120.0` | mm/s, hard cap applied by WorkspaceGuard|
| `robot.max_ang_speed`   | `28.0`  | deg/s                                  |
| `robot.max_lin_accel`   | `650.0` | mm/s²                                  |
| `robot.max_ang_accel`   | `160.0` | deg/s²                                 |

### Workspace envelope (disabled by default)

| Key                       | Default     | Meaning                                          |
|---------------------------|-------------|--------------------------------------------------|
| `workspace.enabled`       | `false`     | Master switch                                    |
| `workspace.x_min / x_max` | `-300 / 400`| mm, BASE                                         |
| `workspace.y_min / y_max` | `200 / 600` |                                                  |
| `workspace.z_min / z_max` | `150 / 600` |                                                  |
| `workspace.rx_range`      | `45.0`      | ±deg around safe_rx (i.e. W)                     |
| `workspace.ry_range`      | `45.0`      | ±deg around safe_ry (i.e. P)                     |
| `workspace.rz_range`      | `90.0`      | ±deg around safe_rz (i.e. R)                     |

### Leap Motion / gesture

| Key                              | Default  | Meaning                                                              |
|----------------------------------|----------|----------------------------------------------------------------------|
| `gesture.grab_closed_threshold`  | `0.85`   | LeapC `grab_strength` ≥ this → posture Closed                        |
| `gesture.grab_open_threshold`    | `0.30`   | ≤ this → posture Open. Keep ≥0.4 hysteresis from the closed value    |
| `gesture.min_confidence`         | `0.3`    | Reject samples below                                                 |
| `gesture.hand_loss_timeout_s`    | `0.15`   | Grace before declaring a hand lost                                   |
| `gesture.posture_hold_s`         | `0.10`   | Debounce on engagement posture                                       |

### Hand-to-robot mapping (smoothing / scaling)

| Key                                   | Default | Meaning                                                              |
|---------------------------------------|---------|----------------------------------------------------------------------|
| `motion.position_scale`               | `0.90`  | Gain applied by the state machine: cmd.linear_velocity = scale * raw |
| `motion.orientation_scale`            | `0.45`  | Same for rotation                                                    |
| `motion.position_deadzone_mm`         | `1.5`   | Hand-displacement dead-zone before any motion                        |
| `motion.orientation_deadzone_deg`     | `1.5`   |                                                                      |
| `motion.smoothing_alpha`              | `0.14`  | EMA on hand samples (0 = no smoothing, 1 = infinite)                 |
| `motion.sign_x..sign_rz`              | `1`     | Per-axis sign flips for venue calibration                            |

In pursuit mode the effective hand→robot gain is
`position_scale * micro_hand_to_robot_ratio` ≈ `0.90 * 0.60 = 0.54`.

### Gripper gesture

| Key                              | Default   | Meaning                                                  |
|----------------------------------|-----------|----------------------------------------------------------|
| `gripper.distance_open_mm`       | `170.0`   | Hand separation ≥ → OPEN impulse                         |
| `gripper.distance_close_mm`      | `50.0`    | ≤ → CLOSE impulse                                        |
| `gripper.neutral_min/max_mm`     | `80/140`  | Must re-enter this band between commands to re-arm       |
| `gripper.facing_dot_max`         | `-0.5`    | Palm-normal dot product threshold (more negative=stricter)|
| `gripper.cooldown_s`             | `0.8`     | Min interval between any two gripper impulses            |
| `gripper.gesture_hold_s`         | `0.25`    | Posture hold before gripper-gesture armed                |
| `gripper.open_do_index`          | `2`       | Tool DO channel for OPEN coil                            |
| `gripper.close_do_index`         | `1`       | Tool DO channel for CLOSE coil                           |

### Micro-motion supervisor (scheduler + per-command bounds)

| Key                              | Default | Meaning                                                              |
|----------------------------------|---------|----------------------------------------------------------------------|
| `robot.micro_command_rate_hz`    | `10.0`  | Robot command issue rate (10 amovels per second)                     |
| `robot.micro_min_period_s`       | `0.10`  | Hard floor on the period between two amovels                         |
| `robot.micro_max_delta_xyz_mm`   | `22.0`  | Legacy incremental cap (used when pursuit OFF)                       |
| `robot.micro_max_delta_rot_deg`  | `4.0`   |                                                                      |
| `robot.micro_deadband_mm`        | `0.5`   | Legacy incremental deadband                                          |
| `robot.micro_deadband_deg`       | `0.3`   |                                                                      |
| `robot.micro_lin_vel`            | `120.0` | mm/s passed to amovel                                                |
| `robot.micro_ang_vel`            | `28.0`  | deg/s                                                                |
| `robot.micro_lin_acc`            | `650.0` | mm/s²                                                                |
| `robot.micro_ang_acc`            | `160.0` | deg/s²                                                               |

### Blending (between consecutive amovel segments)

| Key                              | Default      | Meaning                                                        |
|----------------------------------|--------------|----------------------------------------------------------------|
| `robot.micro_blending_enabled`   | `true`       | Master switch                                                  |
| `robot.micro_blending_radius_mm` | `8.0`        | Requested radius (capped at 0.4 * segment_norm at run time)    |
| `robot.micro_blending_type`      | `duplicate`  | `duplicate` (smooth velocity continuity) or `override`         |

### Pursuit / lookahead controller

| Key                                | Default | Meaning                                                          |
|------------------------------------|---------|------------------------------------------------------------------|
| `robot.micro_pursuit_enabled`      | `true`  | Master switch                                                    |
| `robot.micro_hand_to_robot_ratio`  | `0.60`  | Effective hand→robot motion ratio (on top of position_scale)     |
| `robot.micro_min_step_xyz_mm`      | `5.0`   | Pursuit step lower bound (legacy step path only)                 |
| `robot.micro_max_step_xyz_mm`      | `22.0`  | Pursuit step upper bound                                         |
| `robot.micro_min_step_rot_deg`     | `0.8`   |                                                                  |
| `robot.micro_max_step_rot_deg`     | `4.0`   |                                                                  |
| `robot.micro_arrival_band_xyz_mm`  | `2.0`   | Below this error, no step                                        |
| `robot.micro_arrival_band_rot_deg` | `0.5`   |                                                                  |

### Velocity smoothing layer (NEW in `2ae2e53`)

| Key                                  | Default   | Meaning                                                                   |
|--------------------------------------|-----------|---------------------------------------------------------------------------|
| `robot.micro_velocity_filter_enabled`| `true`    | Master switch. False = pure position-step pursuit (legacy path).          |
| `robot.micro_velocity_filter_alpha`  | `0.35`    | Low-pass filter on desired velocity (1=no filter, 0=frozen)               |
| `robot.micro_max_jerk_xyz`           | `3000.0`  | mm/s³                                                                     |
| `robot.micro_max_jerk_rot`           | `800.0`   | deg/s³                                                                    |
| `robot.micro_velocity_deadband_mm_s` | `1.5`     | Below this filtered velocity, treat as zero and skip the amovel           |
| `robot.micro_stop_ramp_time_s`       | `0.18`    | Duration of the velocity ramp-to-zero on falling edge active→passive     |

### Button / UI / logging / dry-run

| Key                         | Default                       | Meaning                                                |
|-----------------------------|-------------------------------|--------------------------------------------------------|
| `button.mode`               | `keyboard`                    | (only keyboard implemented)                            |
| `button.keyboard_key`       | `SPACE`                       | Toggle authorise                                       |
| `ui.clear_screen_each_tick` | `true`                        |                                                        |
| `ui.show_hand_debug`        | `false`                       |                                                        |
| `log.level`                 | `INFO`                        | `TRACE / DEBUG / INFO / WARN / ERROR`                  |
| `log.file`                  | `doosan_gesture_demo.log`     |                                                        |
| `dryrun.robot`              | `false`                       | Skip real DRFL writes; integrate twist in simulator    |
| `dryrun.gripper`            | `false`                       |                                                        |

### Run-time tuning hints

- More smoothness: lower `micro_velocity_filter_alpha` (0.35 → 0.20),
  lower `micro_max_jerk_xyz` (3000 → 1500).
- More snappiness: raise `alpha` (0.35 → 0.55), raise `jerk` (3000 → 6000).
- Reduce step-jerkiness without changing feel: raise
  `micro_command_rate_hz` (10 → 14) to update the velocity controller more
  often; the scheduler floor `micro_min_period_s` should be ≤ 1/rate.
- Emergency rollback (no recompile) if a 5.7056 reappears:
  ```ini
  robot.micro_velocity_filter_enabled = false   # back to position-step pursuit
  robot.micro_blending_enabled        = false   # OVERRIDE: independent segments
  robot.micro_blending_radius_mm      = 0.0
  ```

---

## 8. What has already been tried and should NOT be repeated

1. **Streaming `speedl()` at 60 Hz.** Tripped 5.7056 immediately. Replaced
   by discrete `amovel` commands at 10 Hz. Do not reintroduce the speedl
   streaming path on the active loop.
2. **Decel ramp via `speedl(twist * factor)` on falling edge.** Was
   superseded by the amovel-only design + ramp-to-zero via velocity
   smoothing. The remaining `kDecelFactors` constant in `Application.cpp`
   is dead code reachable only if both the velocity filter AND the
   pursuit controller are disabled — leave it for now, but do NOT
   re-wire it into the active path.
3. **`mwait()` to wait for motion completion.** Triggers 5.7056. Polling
   `STATE_STANDBY` after `movejx` is the documented workaround used by
   `moveHome`.
4. **Calling `getCurrentPose()` every tick (for workspace guard or UI).**
   Caused `CONTROL_CHECK_CURRENT_TASK_POSITION` interleaving and 5.7056.
   The active path now uses `WorkspaceGuard::clampSpeed()` (pose-less);
   the UI uses a cache refreshed only when `!cur_active`.
5. **`emergencyStop()` (STOP_TYPE_QUICK) on shutdown, disengage, passive
   transitions, sensor-disconnect.** Dropped the servo into SAFE_OFF
   then "mastering lost" next session. Restricted to critical
   `RobotError` / `InternalError` only.
6. **Naming the rotation axes as extrinsic XYZ.** Doosan V3.5 uses
   intrinsic ZYZ' (W, P, R). The `rx / ry / rz` field names are
   historical; treat them as W/P/R. The convention is documented in
   `src/robot/RobotPose.hpp` and in §26 of the context brief.
7. **`movel` to a wrist-singular safe pose.** Trips alarm 3205 →
   SAFE_OFF 7056. Use `movejx` for the home approach
   (`home_use_movejx = true`) and AVOID singularity handling.
8. **`STOP_TYPE_QUICK` in `stopMotion()`.** `stopMotion()` is the soft
   path (`speedl(0, ..., 0.2 s)`); only `emergencyStop()` issues
   `STOP_TYPE_QUICK`.
9. **Pushing to a different branch without explicit user permission.**
   The current branch is `codex/add-configurable-blending-for-micro-movements`.
   The previous branch was `claude/gesture-control-doosan-robot-qme86`.
   Do not switch without checking with the user.
10. **Guessing DRFL function signatures.** The signatures of `amovel`,
    `change_collision_sensitivity`, `set_singularity_handling`,
    `set_safety_mode`, `manage_access_control`, etc. were discovered
    empirically and through inspection of the public header on GitHub.
    When in doubt, `WebFetch` the actual header at
    `https://raw.githubusercontent.com/DoosanRobotics/API-DRFL/main/include/DRFLEx.h`.

---

## 9. Recommended next steps

### Immediate

1. **Validate on hardware** that the velocity-smoothing layer eliminates
   the residual jerkiness reported on commit `4399554`. Look at:
   - Does the motion feel continuous (no audible / visible step accel)?
   - Does `LOG_D` `vel-smooth: err=... v=... a=... seg=... r=...` show
     a continuous velocity profile (filtered_velocity_ changing smoothly
     rather than jumping)?
   - Does the falling-edge ramp produce ~3–4 tail amovels with shrinking
     velocity (look for `Velocity ramp-to-zero complete`)?
2. **If a 5.7056 still fires**: capture the DRFL alarm tuple
   (`level / group / index / params`) from the log AND the controller's
   pendant popup. With the current architecture, a 5.7056 would almost
   certainly mean the velocity-filter math produced a `next_target` that
   created an effective joint accel above the safety supervisor's limit.
   The diagnostic plan in that case is:
   - Check `LOG_D vel-smooth` lines around the trip — what was the
     commanded velocity / accel?
   - Compare to `robot.max_lin_accel` and `robot.max_ang_accel` and the
     pendant's joint-accel limit page.
   - Lower the jerk caps before lowering anything else.

### Short-term

3. **Tune live**: try the three profiles annotated in `demo_config.ini`
   (conservative / balanced / responsive) and have the operator pick.
4. **Re-enable collision detection** (`robot.collision_sensitivity = 50`)
   once the operator has declared a realistic payload and TCP on the
   pendant. Currently 0 because we cannot match the inverse-dynamics
   prediction without that information.
5. **Re-enable the workspace envelope** once the cell has been walked.
   Set `workspace.enabled = true` with a small cube around the safe pose,
   then widen.

### Medium-term

6. **Telemetry**: add a CSV or JSON metric stream
   (per-tick `cmd.linear_velocity`, `filtered_velocity_`, segment lengths,
   alarms) so the demo run can be replayed for tuning between sessions.
7. **Operator UI niceties**: a colored ribbon for the demo state, a hand
   schematic, and an alarm banner that surfaces the most recent
   `LOG_E DRFL alarm: ...` directly in the console.
8. **PR / branch cleanup**: a follow-up PR could fold the velocity
   smoothing layer onto `main` once it has been validated. The other
   pursuit-related changes (`4399554` / `2126605` / `cb8361d` /
   `97883c1`) are already integrated.

### Do NOT pursue without an explicit user request

- Switching to DRFL RT control / `connect_rt`.
- Reintroducing continuous `speedl` streaming.
- Adding `mwait()` anywhere.
- Calling `getCurrentPose()` during active streaming.
- Calling `STOP_TYPE_QUICK` outside `emergencyStop()`.
- Force-pushing or rewriting history on the codex branches.

---

## 10. Quick smoke check (any new commit)

Before pushing, run:

```bash
cmake --build build && \
build/state_machine_smoke_test
```

The 11-scenario smoke test must end with
`All state machine smoke tests passed.`

Then run the audit greps from §5 above and ensure the only `getCurrentPose`
call sites in `src/app/` are the active-entry seed and the throttled UI
refresh.

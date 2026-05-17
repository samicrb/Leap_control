# Context File — Doosan Open House Gesture-Control Demo (Leap Motion + DRFL)

## 1. Document Purpose

This document is a **functional and technical context brief** intended for a code-generation AI such as **Claude Code** or **Codex**.

Its role is to define the project clearly enough for implementation **without ambiguity on the main behavior**, while still leaving reasonable implementation freedom for software structure.

This project is a **demo-only system** for an **open house event**. It is **not** intended to become a production application.

The implementation priority is:

1. **Robustness for event use**
2. **Perceived responsiveness**
3. **Smooth and intuitive interaction**
4. **Simple operator experience**
5. **Visual impact**

---

## 2. Demo Objective

The demo must showcase:

- the **reactivity** of the **Doosan DRFL API**
- the **fluidity** of real-time external control
- the image of **AI-powered robotics**
- a strong feeling that the visitor is **directly driving the robot with hand gestures**

The demo must feel:

- **spectacular**
- **interactive**
- **immediately understandable after a short briefing**
- **safe and constrained**
- **repeatable all day**

There is **no requirement to display metrics** such as latency or update rate. The goal is the **visual impression** that the robot moves instantly and naturally.

---

## 3. Platform and Environment

### Robot
- **Robot model:** Doosan **M1013** (project history: A0912 -> A0509 -> M1013;
  the platform was retargeted twice during bring-up. M-series has a longer
  reach than the cobot A-series, behaviour is otherwise comparable.)
- **Controller software:** **V3.5**
- **Doosan robot API version to use:** **DRFL 1.33.3** (from
  [github.com/DoosanRobotics/API-DRFL](https://github.com/DoosanRobotics/API-DRFL))
- **Robot IP / port:** `192.168.1.25` : `12345` (configurable in
  `config/demo_config.ini`)
- **Safe pose (BASE frame, Doosan ZYZ' Euler):**
  `x=55, y=400, z=375, rx=33 (W), ry=96 (P), rz=110 (R)` in mm / deg.
  `rx/ry/rz` are stored under those legacy names but are interpreted as
  Doosan `(W, P, R)` — see `src/robot/RobotPose.hpp`.
- **End effector:** any **two-state, tool-DO–driven gripper**
  (Schunk, Robotiq, OnRobot, custom). Wired through
  `src/gripper/ToolIoGripperController.{hpp,cpp}` — channel indices are
  configurable (`gripper.open_do_index`, `gripper.close_do_index`).

### Sensor
- **Leap Motion Controller** (any USB unit supported by Ultraleap Gemini)
- **Ultraleap SDK / tracking stack to use:** **Gemini 6.2.0**
- **Preferred API for sensor integration:** **LeapC**
- Mounted **on a fixed table**
- Sensor oriented **upward**
- Controlled interaction area is in front of the visitor

### Host PC
- External PC connected to the robot
- **Dell Latitude 5550**
- **Windows 11**
- Only a **basic GPU** is available
- The solution must **not depend on a powerful GPU**
- The solution must **not depend on internet access**

### Event conditions
- Visitors are **briefed beforehand**
- Visitors are **guided by an operator/animator**
- Lighting is **controlled**
- Robot is integrated in a cell **close to the public**, so behavior must remain conservative and easy to supervise

---

## 4. Project Scope

## In scope
- real-time gesture-based robot control
- relative Cartesian translation control
- relative TCP orientation control
- gripper open/close control through a dedicated two-hand gesture
- simple operator UI
- state machine for reliable interaction
- safety-oriented fallback behavior
- short visible pick-and-place scenario

## Out of scope
- advanced AI/ML training
- cloud services
- gesture personalization
- complex motion planning
- production-grade teleoperation
- autonomous scene understanding
- multi-user support
- voice interaction
- long-term maintainability beyond demo quality

---

## 5. High-Level Interaction Concept

The demo is a **two-hand interaction system**.

### Core idea
- The **right hand** is the main control hand.
- The **left hand** acts as a **mode / authorization hand**.
- A physical **external button** enables the demo session.
- The robot is controlled in a **small, tightly constrained workspace**.
- The user can pick, move, and place a light demo object.

### Main user experience
The visitor should feel:
- “I am moving the robot myself”
- “The robot reacts immediately”
- “This is intuitive and futuristic”

---

## 6. Functional Control Philosophy

## 6.1 General control style
Control is **relative**, not absolute.

The hand acts like a **3D joystick**:
- hand displacement drives robot displacement incrementally
- there is no absolute mapping between hand position and robot pose

This choice is intentional because it is:
- more robust
- easier to keep safe
- easier to use in a small event workspace
- less sensitive to limited arm reach

## 6.2 Re-centering concept
The visitor must be able to move the robot farther than the natural physical reach of the arm.

Therefore:
- control is active when the right hand is in the correct posture
- if the user opens the right hand, robot motion freezes
- the user can then reposition the right hand near the center of the Leap Motion interaction space
- once the correct control posture is re-established, control resumes

This re-centering behavior is a **deliberate feature**, not an error.

---

## 7. Intended Demo Scenario

The physical scenario should be a **simple visible pick-and-place**.

Recommended scenario:
1. Start demo
2. User takes control of the robot
3. Robot is guided toward a **light object**
4. User opens the gripper
5. User closes the gripper to grasp the object
6. User moves the object to another location
7. User opens the gripper to release it
8. Session ends or returns to ready state

### Object type
Preferred object type:
- **lightweight branded demo object** if available

Fallback options:
- **foam cube**
- **lightweight part**

The object must be:
- safe
- visually readable from a distance
- easy to grip
- forgiving if grasp is slightly imperfect

---

## 8. Workspace Philosophy

The robot workspace used by the demo must be:

- **small**
- **highly controlled**
- visually easy to understand
- safer than the full robot reachable volume

The implementation must define a **demo workspace envelope** smaller than the robot's full capabilities.

The brief does not impose exact dimensions, but the intent is:
- small enough to remain safe and readable
- large enough to demonstrate obvious motion in X/Y/Z
- suitable for pick-and-place of a lightweight object

A practical implementation should use:
- bounded Cartesian limits
- bounded orientation limits
- bounded speed
- bounded acceleration / aggressiveness
- a neutral or home-like pose for recovery

---

## 9. Gesture Logic Overview

## 9.1 External start condition
A **first external button** enables or launches the demonstration.

Without this button being active:
- no gesture-driven motion is allowed

This button acts as a supervisor/operator permission.

## 9.2 Hands and roles
- **Left hand** = authorization / mode selection
- **Right hand** = main motion control hand

## 9.3 Right hand motion control posture
The right hand is tracked primarily when it is in a **closed fist** posture.

This posture is used to:
- reduce ambiguity
- make “control active” visually clear
- support the re-centering logic

## 9.4 Re-centering posture
If the user opens the right hand:
- robot motion must freeze immediately
- user can reposition the right hand
- motion resumes only after the proper engagement posture is recovered

## 9.5 Gripper gesture
Gripper control uses a dedicated **two-hand gesture**:
- both hands oriented roughly like a clapping posture (facing each other, approximately perpendicular to the sensor plane)
- then the distance between hands drives open / close events

Thresholds:
- hand distance **> 170 mm** → **open gripper**
- hand distance **< 50 mm** → **close gripper**

This must be implemented as **impulse commands**, not continuous repeated toggling.

---

## 10. Mode Logic

The left hand determines the control mode.

## 10.1 Position mode
If the **left hand is present and open**, the system is in **position mode**.

In position mode:
- the relative motion of the **right hand** controls **relative Cartesian translation**
- intended axes: **X / Y / Z**

## 10.2 Orientation mode
If the **left hand is present and closed**, the system is in **orientation mode**.

In orientation mode:
- the **relative orientation change of the right hand** controls **relative TCP orientation**

Interpretation:
- translation and orientation are not driven simultaneously in the same instant
- the left hand selects which family of control is active

This split is intentionally chosen to keep the interaction understandable and robust.

---

## 11. Canonical Behavior Rules

The following behavior rules must be treated as **authoritative**.

### Rule 1 — No motion without external authorization
If the external demo button is not active:
- the robot must not respond to gestures

### Rule 2 — Right hand closed fist is required for active robot driving
The system should only drive robot motion when:
- the correct mode conditions are met
- the relevant hands are tracked
- the right hand is in the required active posture

### Rule 3 — Loss of required condition causes immediate freeze
If any required control condition disappears:
- robot motion must stop immediately
- the system must transition to a safe hold / ready logic

### Rule 4 — Left hand is mandatory
If the left hand disappears while control is active:
- freeze immediately

### Rule 5 — Tracking loss does not auto-resume blindly
After tracking loss or invalid condition:
- transition back to **Ready**
- require the user to re-establish the engagement posture

### Rule 6 — Ambiguous gestures must be ignored
If the input is ambiguous:
- do not guess
- do not move
- keep or return to a safe state

### Rule 7 — Gripper control suspends motion control
When gripper gesture mode is recognized:
- Cartesian/orientation driving is temporarily suspended
- the gripper action is executed
- then the system returns to a non-dangerous interaction state (Ready or active state, depending on implementation choice)

Recommended default:
- return to **Ready** after a gripper action, so that the user must re-engage deliberately

### Rule 8 — Slight smoothing is allowed
A **very light smoothing/filtering** is acceptable if it does not visibly damage responsiveness.

The user explicitly wants:
- motion to feel immediate
- no heavy smoothing
- no sluggish behavior

---

## 12. State Machine

The implementation must use a clear state machine.

Recommended state machine:

### 12.1 Idle
Conditions:
- external button not active

Behavior:
- no gesture control
- robot remains still
- UI indicates demo inactive

### 12.2 Ready
Conditions:
- external button active
- system waiting for valid engagement posture

Behavior:
- no robot motion
- UI explains how to engage control

### 12.3 Position Control Active
Conditions:
- left hand present and **open**
- right hand present and **closed**
- tracking valid
- robot state compatible
- all demo safety conditions valid

Behavior:
- right-hand relative motion controls robot translation

### 12.4 Orientation Control Active
Conditions:
- left hand present and **closed**
- right hand present and **closed**
- tracking valid
- robot state compatible
- all demo safety conditions valid

Behavior:
- right-hand relative orientation controls TCP orientation

### 12.5 Recenter / Hold
Conditions:
- left hand valid for current mode
- right hand intentionally opened or not in active posture
- no fault condition

Behavior:
- freeze robot
- allow user to reposition right hand
- wait for valid re-engagement posture

### 12.6 Gripper Mode
Conditions:
- dedicated two-hand gripper gesture is recognized

Behavior:
- suspend robot motion control
- interpret hand distance
- issue one gripper open/close command when threshold is crossed
- prevent repeated triggering from the same held posture
- exit to Ready (recommended) or controlled re-engagement flow

### 12.7 Fault / Hold
Conditions may include:
- tracking lost
- left hand missing
- right hand tracking invalid
- robot state incompatible
- workspace/safety constraint violated
- internal error

Behavior:
- immediate freeze
- UI indicates hold/fault reason if available
- transition to Ready when conditions are cleared
- require deliberate re-engagement

---

## 13. Engagement Logic

A practical engagement sequence should be:

1. External button enables demo
2. System enters **Ready**
3. User presents both hands
4. User selects mode with left hand:
   - left hand open = position mode
   - left hand closed = orientation mode
5. User closes right hand into the active control fist
6. System captures a motion/orientation reference
7. Active control begins

Important:
- engagement should be deliberate
- accidental hand presence must not drive the robot
- reference values should be refreshed when re-engaging after recentering

---

## 14. Motion Mapping Requirements

## 14.1 Translation
In position mode:
- relative displacement of the right hand must drive relative robot translation
- axes exposed: **X / Y / Z**

Expected behavior:
- intuitive and consistent
- no sudden jumps when re-engaging
- constrained to the demo workspace
- conservative speed limits

## 14.2 Orientation
In orientation mode:
- relative orientation changes of the right hand drive relative TCP orientation
- implementation should choose the most stable representation available
- orientation must remain bounded and readable

Priority:
- robustness over sophistication
- avoid orientation behaviors that feel erratic or overly sensitive

## 14.3 Speed feeling
The control must feel:
- immediate
- smooth
- not overly damped
- not aggressive

Relative hand motion scaling should be tuned for:
- good visible motion
- comfortable visitor control
- low risk of abrupt robot motion

---

## 15. Gripper Gesture Requirements

Gripper control is deliberately separated from motion control.

### Required posture
- two hands visible
- both hands approximately facing each other in a clap-like arrangement

### Distance thresholds
- distance > **170 mm** → trigger **open**
- distance < **50 mm** → trigger **close**

### Command style
Use **single-shot/impulse behavior**, not continuous command spam.

### Debounce / anti-repeat behavior
The implementation must avoid repeated open or close triggers while the user maintains the same posture.

Recommended approach:
- only trigger when a threshold is crossed
- require returning to a neutral zone before another gripper command can be issued

### Priority
When gripper gesture is active:
- suspend robot motion control first
- then evaluate gripper action

---

## 16. Safety and Conservative Constraints

This is still a demo in public context. Conservative behavior is mandatory.

### Required safety philosophy
- reduced speed
- limited workspace
- no sudden jumps
- immediate freeze on invalid tracking
- immediate freeze when left hand disappears
- immediate freeze when conditions are not met
- ignore ambiguous input

### Conservative implementation expectations
- bounded command rates
- bounded motion increments
- no dependence on high-frequency unstable behavior
- no uncontrolled drift
- no automatic recovery motion without clear logic
- avoid any control mode that can produce jerk or snap behavior

### Recovery philosophy
After an invalid condition:
- hold
- return to Ready
- require re-engagement

Do not resume active motion silently.

---

## 17. User Interface Requirements

The UI must remain **very simple** and public-friendly.

Display only the following:
- **system mode**
- **tracking status**
- **recognized gesture / action**
- **gripper status**
- **short instruction / prompt**

Avoid crowded debugging information in the main event UI.

Example categories:
- Demo inactive
- Waiting for button
- Ready
- Position mode
- Orientation mode
- Recenter hand
- Gripper open
- Gripper close
- Tracking lost
- Hold

Optional:
- small robot status indicator if it improves supervision

Not required:
- raw sensor values
- latency charts
- developer logs on main screen

---

## 18. Error Handling Requirements

The implementation must explicitly handle:

- no sensor detected
- sensor initialized but no hands visible
- only one hand visible
- left hand missing during active control
- right hand posture invalid
- gesture ambiguity
- gripper gesture partially matched
- robot communication issue
- robot state not compatible with control
- workspace limit reached
- failed gripper action
- unexpected internal exception

Preferred behavior in most cases:
- stop motion safely
- inform the user/operator briefly on screen
- return to Ready when possible

---

## 19. Practical Assumptions Chosen to Resolve Ambiguity

The user explicitly allowed sensible decisions in ambiguous areas. The following assumptions should therefore be treated as part of the spec.

### Assumption A — Two-hand demo is official
Although earlier exploration mentioned one hand, the final concept is explicitly **two-handed**.

### Assumption B — Right hand fist is the active control posture
This is the canonical active posture for motion/orientation control.

### Assumption C — Right hand opening means hold/recenter, not control
Opening the right hand is interpreted as a deliberate pause for repositioning.

### Assumption D — Left hand defines mode
- left open = position mode
- left closed = orientation mode

### Assumption E — Gripper command should return to Ready
After a gripper open/close command, the safest and clearest behavior is to return to **Ready** and require deliberate re-engagement.
This may be changed only if implementation testing shows a smoother but equally safe alternative.

### Assumption F — Demo workspace is intentionally small
The application must not try to impress through large reach. It must impress through responsiveness and clarity.

### Assumption G — Robustness is more important than maximal feature richness
If a feature creates instability, remove or simplify it.

---

## 20. Implementation Guidance for the Code-Generation AI

The code-generation AI should produce a solution that is:

- modular
- easy to tune
- event-robust
- understandable by engineers
- easy to demo repeatedly

Recommended internal separation:
- sensor acquisition layer
- hand interpretation / gesture detection layer
- state machine layer
- robot command layer
- UI/status layer
- configuration layer

A configuration section should exist for:
- hand distance thresholds
- workspace limits
- motion/orientation scaling
- smoothing parameters
- timeouts
- gripper logic debounce values
- safe/default poses
- UI messages
- enable/disable flags for test modes

---

## 21. Testing Expectations

The implementation should be testable with explicit scenarios.

Minimum required test scenarios:

### Sensor / gesture tests
1. No hands visible
2. Left hand only visible
3. Right hand only visible
4. Valid position engagement
5. Valid orientation engagement
6. Right-hand recenter behavior
7. Gripper open gesture
8. Gripper close gesture
9. Ambiguous gesture ignored
10. Tracking loss during motion
11. Left-hand disappearance during motion

### Robot behavior tests
12. No motion when button inactive
13. No motion when engagement posture incomplete
14. Relative translation feels intuitive
15. Orientation mode is stable and bounded
16. No motion jump on re-engagement
17. Motion stops immediately on invalid condition
18. Workspace limits respected
19. Gripper action suspends motion control
20. System returns to Ready after fault/hold recovery

### Demo scenario tests
21. Full pick-and-place cycle with light object
22. Repeated sessions with different users
23. Long event usage stability
24. Cold start to demo-ready workflow
25. Operator-supervised recovery after abnormal input

---

## 22. Acceptance Criteria

A first version should be considered successful if:

1. The system starts reliably on the event PC
2. The Leap Motion sensor works without special GPU requirements
3. The external button correctly gates the demo
4. Visitors can understand the interaction after a short briefing
5. The robot feels reactive and smooth
6. The robot never moves unexpectedly
7. Tracking loss causes immediate freeze
8. Re-engagement is easy and predictable
9. Gripper open/close works reliably enough for a visible demo
10. A lightweight object can be picked, moved, and released in a controlled way
11. The demo can be repeated many times without complex recalibration
12. The UI remains clear and minimal

---

## 23. Priority Order for Trade-Offs

If trade-offs are needed, obey this priority order:

1. **Safety and predictability**
2. **Event robustness**
3. **Clear user experience**
4. **Perceived responsiveness**
5. **Visual appeal**
6. **Feature richness**

Examples:
- Remove unstable orientation features before sacrificing reliability
- Simplify gesture recognition before accepting false positives
- Reduce speed before accepting abrupt motion
- Simplify the scenario before increasing operational risk

---

## 24. Fixed Technical Version Requirements

The following version choices are fixed project constraints and must be respected unless a real hardware compatibility issue forces a change:

- **Ultraleap tracking stack / SDK:** **Gemini 6.2.0**
- **Ultraleap sensor API:** **LeapC**
- **Doosan robot API:** **DRFL 1.33.3** (API-DRFL GitHub release)

Implementation note:
- Do not use a legacy Leap Motion SDK as the primary integration path.
- Do not target another DRFL version unless compatibility testing on the actual event setup makes it strictly necessary.

## 24. Preferred References

For Doosan API behavior, prefer the official DRFL API manual:
- https://doosanrobotics.github.io/doosan-robotics-api-manual/

For implementation decisions, prefer:
- official documentation
- predictable and stable approaches
- conservative real-time behavior
- simple tunable logic over clever but fragile heuristics

---

## 25. Final Instruction to the Code-Generation AI

Build a **demo-grade, robust, operator-friendly gesture control application** for a **Doosan M1013 with V3.5 controller**, using a **Leap Motion Controller** and an **external Windows 11 PC**, with the following goals:

- make the robot feel reactive and intuitive
- keep behavior safe and conservative
- support a small-workspace pick-and-place demo
- use a clear state machine
- keep the UI simple
- handle loss of tracking safely
- allow deliberate re-engagement
- prioritize reliability over complexity

When uncertain between a flashy behavior and a stable behavior, choose the **stable behavior**.

---

## 26. Implementation Status (post-bring-up)

This section captures what was actually built and what was learned during
hardware bring-up. It supersedes earlier sections when in conflict.

### 26.1 What ships
- 39-file C++17 project, MSVC-clean, builds on the event PC with CMake.
- Real DRFL 1.33.3 adapter (`DrflRobotController`) + LeapC Gemini 6.2.0
  adapter (`LeapSource`). Both fall back to cooperative stubs when their
  SDK is absent, so the state machine / UI can be exercised on a dev
  laptop.
- 11-scenario state-machine smoke test (no hardware needed).
- All tuning knobs in `config/demo_config.ini` — no recompile required.

### 26.2 DRFL bring-up sequence (validated)
The adapter performs this sequence in `connect()` + `engage()`, in order.
Every step came from a real bring-up failure on the cell:

1. `open_connection(ip, port)`
2. `setup_monitoring_version(1)`
3. Register monitoring callbacks: `state`, `access_control`, `log_alarm`
4. `manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)`
5. Wait for `MONITORING_ACCESS_CONTROL_GRANT` (timeout-bounded)
6. **Auto-reset latched safety stops** (`CONTROL_RESET_SAFET_STOP` /
   `_OFF`) — a SAFE_STOP from a previous run latches across sessions
   and masquerades as "mastering lost".
7. If `STATE_SAFE_OFF` / `STATE_SAFE_OFF2`: `set_robot_control(CONTROL_SERVO_ON)`
8. Wait for `STATE_STANDBY`
9. `set_robot_mode(ROBOT_MODE_AUTONOMOUS)`
10. `set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP)` —
    avoids inheriting the pendant's MANUAL collaborative speed cap.
11. `change_collision_sensitivity(0.0f)` — bring-up only; payload / TCP
    aren't declared so model-based collision detection must be off.
12. `set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID)` — let the
    controller blend through wrist singularities; without this the
    safe-pose move trips alarm 3205 → SAFE_OFF (alarm 7056).
13. Now safe to issue motion commands.

A real **mastering loss** (`STATE_RECOVERY`) is NOT bypassable from the
API; the operator must run `Setting → Robot → Mastering → Use existing
mastering data` on the teach pendant.

### 26.3 Motion primitives
- **Safe-pose approach:** `movejx(target, sol=0, vel=20°/s, acc=60°/s²)`
  by default (configurable). `movejx` interpolates in joint space which
  is immune to Cartesian wrist singularities. `movel` is still
  available via `robot.home_use_movejx = false`.
- **Streaming:** `speedl(twist, accel, 0.1)` at the loop rate. Angular
  velocity is interpreted as an instantaneous ω-vector around BASE X/Y/Z
  — small-angle Euler-rate ≈ ω-vector approximation is fine in the
  demo's bounded orientation range.
- **Pause vs. emergency:**
  - `stopMotion()` → soft pause via zero `speedl`. Safe at 60 Hz; used
    in every passive state (IDLE / READY / RECENTER / GRIPPER /
    Fault re-ticks).
  - `emergencyStop()` → `stop(STOP_TYPE_QUICK)`. Reserved for the
    one-shot Fault entry and the shutdown path; can transition the
    controller to SAFE_OFF, so never call it on the streaming loop.

### 26.4 Doosan V3.5 rotation convention
The `RobotPose` fields `rx, ry, rz` are **Doosan ZYZ' intrinsic Euler
(W, P, R)** — not extrinsic XYZ around BASE X/Y/Z. The names are
preserved for historical reasons but documented at every site that
matters (`RobotPose.hpp`, `Config.hpp`, `demo_config.ini`,
`docs/TUNING_GUIDE.md`). Values captured from the pendant's posx
display flow through verbatim.

### 26.5 Configurable flags added during bring-up
All in `config/demo_config.ini` under the `robot.*` namespace:

| Flag | Default | Use |
|---|---|---|
| `robot.skip_move_home` | `false` | bypass the home `movejx` for first-time integration |
| `robot.collision_sensitivity` | `0` | 0 disables; restore 50-75 once payload/TCP declared |
| `robot.singularity_handling` | `0` | 0=AVOID, 1=STOP, 2=VEL (Doosan enum) |
| `robot.home_use_movejx` | `true` | `movejx` joint-space vs. `movel` Cartesian for home |
| `robot.auto_reset_safety` | `true` | clear latched SAFE_STOP / SAFE_OFF on engage |
| `workspace.enabled` | `false` | workspace cube + orientation cone — keep off for bring-up |

### 26.6 Lessons learned
- Doosan controllers latch safety stops across sessions. Symptom looks
  like "mastering lost"; cure is `CONTROL_RESET_SAFET_*` at engage, not
  pendant intervention.
- Collision detection trips on phantom torques when payload / TCP
  isn't declared. Always disable for bring-up.
- Cartesian `movel` paths to a "natural" pendant-captured pose can
  graze a wrist singularity even when both endpoints are well-conditioned.
  Default to `movejx` for any planned approach; reserve `movel` for
  motion that *has* to be Cartesian-linear.
- DRFL's `set_on_log_alarm` is the single most useful diagnostic. Wire
  it from day one (level / group / index / params) — the cell's actual
  failure mode is invisible without it.
- `STOP_TYPE_QUICK` in a 60 Hz loop will eventually drop the servo to
  SAFE_OFF. Reserve it for true faults / shutdown.

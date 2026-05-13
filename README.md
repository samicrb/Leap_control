# Doosan Open-House Gesture-Control Demo

Gesture-driven teleoperation of a **Doosan A0509** (controller V3.5, DRFL 1.33.2)
using an **Ultraleap Leap Motion Controller** with the **Gemini 6.2.0 / LeapC**
tracking stack. Built for a public open-house event: robust, predictable,
operator-friendly.

> The robot model, IP, safe pose, gripper wiring and workspace envelope are
> all driven from `config/demo_config.ini` — porting to another Doosan model
> (A0912, M-series, etc.) is a config-only change.

> **Not a product.** This is a demo-grade skeleton: safety-first, conservative
> speeds, explicit state machine, and clean adapter boundaries so you can swap
> real SDK calls without touching the rest of the code.

---

## 1. Features

- Two-hand interaction:
  - **Left hand** = mode selector (open = position, closed = orientation).
  - **Right hand** = control hand (closed fist = drive, open = freeze/recenter).
  - **Both hands facing + distance** = gripper open/close impulse.
- External supervisor button gates every motion (default: SPACE).
- Optional bounded demo workspace (`workspace.enabled`) in Cartesian + orientation.
- Hard speed / acceleration caps, dead-zones, light EMA smoothing.
- Console UI for operator supervision.
- Explicit state machine: `Idle / Ready / Position / Orientation / Recenter /
  Gripper / Fault`.
- Clean SDK adapters (`ILeapSource`, `IRobotController`, `IGripperController`)
  with cooperative simulator fallbacks when the SDKs aren't installed on the
  build machine.

## 2. Project layout

```
Leap_control/
├── CMakeLists.txt
├── README.md
├── config/
│   └── demo_config.ini            # every runtime knob lives here
├── docs/
│   ├── TUNING_GUIDE.md            # how to calibrate thresholds / scaling
│   └── OPERATOR_CHECKLIST.md      # event-day test + run checklist
├── src/
│   ├── main.cpp
│   ├── app/Application.{hpp,cpp}
│   ├── config/Config.{hpp,cpp}
│   ├── sensor/{ILeapSource,LeapSource,HandFrame}.{hpp,cpp}
│   ├── gesture/{GestureInterpreter,GestureTypes}.{hpp,cpp}
│   ├── state/{StateMachine,States}.{hpp,cpp}
│   ├── robot/{IRobotController,DrflRobotController,WorkspaceGuard,RobotPose}.{hpp,cpp}
│   ├── gripper/{IGripperController,ToolIoGripperController}.{hpp,cpp}
│   ├── input/{IExternalButton,KeyboardButton}.{hpp,cpp}
│   ├── ui/ConsoleUI.{hpp,cpp}
│   └── util/{Logger,MathUtils}.{hpp,cpp}
└── test/
    └── state_machine_smoke_test.cpp
```

## 3. Build (Windows 11, MSVC, event PC)

Prerequisites:
- Visual Studio 2022 + CMake
- Ultraleap Gemini 6.2.0 installed (`LeapC.h`, `LeapC.lib`)
- Doosan DRFL 1.33.x SDK from
  [github.com/DoosanRobotics/API-DRFL](https://github.com/DoosanRobotics/API-DRFL)
  (provides `include/DRFLEx.h` and `library/Windows/64bits/DRFLWin64.lib`)

```powershell
git clone <this repo>
cd Leap_control
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
    -DLEAPC_SDK_ROOT="C:\Program Files\Ultraleap\LeapSDK" ^
    -DDRFL_SDK_ROOT="C:\Doosan\DRFL"
cmake --build build --config Release
```

`DRFL_SDK_ROOT` should point at the root of the
[API-DRFL](https://github.com/DoosanRobotics/API-DRFL) checkout — CMake
expects to find `include/DRFLEx.h` and
`library/Windows/64bits/DRFLWin64.lib` underneath it (older 1.33.2
bundles using `lib/x64/DRFL.lib` are also accepted).

The binary ends up at `build/Release/doosan_gesture_demo.exe`. The CMake
post-build step copies `config/demo_config.ini` and the DRFL runtime
DLLs (`DRFLWin64.dll`, `PocoFoundation64.dll`, `PocoNet64.dll`) next to
the binary so it can launch without manual PATH gymnastics.

To build **without** the SDKs (dev laptop / CI), just omit the two
`-D...SDK_ROOT` flags. The build falls back to cooperative stubs:
- sensor always reports "disconnected",
- robot adapter integrates commanded twists locally so the UI / state machine
  can be exercised,
- gripper logs actions instead of pulsing tool DO.

## 4. Run

```powershell
cd build\Release
doosan_gesture_demo.exe --config demo_config.ini
```

Controls in the console window:
- **SPACE** — toggle the supervisor button (demo authorisation).
- **Q / ESC** — quit cleanly.

On first launch, the robot executes a blocking `movel` to the **safe pose**
defined in `demo_config.ini`. The demo does nothing else until SPACE is
pressed **and** both hands are detected in the right posture.

## 5. Robot connection

Defaults shipped: `robot.ip = 192.168.1.2`, `robot.model = A0509`, safe pose
`x=55 y=400 z=375 rx=33 ry=96 rz=110` (mm/deg, BASE frame). Edit any of those
plus `robot.port` in `config/demo_config.ini` if the cell differs. Keep
`dryrun.robot = true` until you have walked the cell and confirmed nothing
else will move during the gesture.

The demo workspace envelope is **disabled by default** (`workspace.enabled
= false`). Only the hard speed / acceleration caps are active. Turn it on
and tune the box once the cell is characterised — see
`docs/TUNING_GUIDE.md`.

## 6. Hardware integration points

Every place in the code where real SDK calls must be wired is tagged with one
of:

- `// DRFL:` — lines in `src/robot/DrflRobotController.cpp`.
- `// DRFL-INTEGRATION:` — wiring for tool DO / gripper.
- `HAVE_LEAPC` / `HAVE_DRFL` compile-guards.

## 7. Tests

A headless smoke test verifies the major state transitions without needing
any hardware:

```powershell
cmake --build build --config Release --target state_machine_smoke_test
build\Release\state_machine_smoke_test.exe
```

See `docs/OPERATOR_CHECKLIST.md` for the full demo-day test protocol.

## 8. Safety reminder

Public demo. The code is conservative on purpose. Do **not**:
- raise `robot.max_lin_speed` above ~200 mm/s without safety review,
- enlarge `workspace.*` before you know what the cell permits,
- leave `workspace.enabled = false` once the cell is fully characterised,
- disable smoothing or dead-zones for a "cooler" feel,
- skip the external button gate.

---

License: internal demo, all rights reserved.

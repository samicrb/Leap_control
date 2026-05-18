# qbRobotics qbAPI SDK drop-in

The `QbSoftClawGripper` adapter (in `src/gripper/`) wraps the official
qbRobotics C/C++ API published by NMMI / qbRobotics:

- Repository: https://github.com/NMMI/qbAPI
- License: per upstream (typically BSD-3-Clause; check the repository
  before redistribution).

When the SDK sources are not present, the build still succeeds and
`QbSoftClawGripper` is compiled as a clearly-logged stub - the
`GripperFactory` then falls back to `NoopGripper` when
`[gripper].required = false`.

## How to enable real SoftClaw control

```
cd external
git clone https://github.com/NMMI/qbAPI.git qbrobotics_sdk
```

After the clone the layout that CMake recognises is one of:

```
external/qbrobotics_sdk/qbmove_communications.h           (top-level)
external/qbrobotics_sdk/src/qbmove_communications.{h,cpp} (NMMI default)
external/qbrobotics_sdk/include/qbmove_communications.h   (custom)
```

The CMake configuration auto-detects either layout (see the
`QBROBOTICS_SDK_ROOT` cache variable in the top-level `CMakeLists.txt`).
The accompanying `.cpp` source file is compiled into the demo binary
when present.

To point CMake at the SDK living elsewhere on disk:

```
cmake -B build -S . -DQBROBOTICS_SDK_ROOT="C:/Users/you/dev/qbAPI"
```

The build emits `HAVE_QBROBOTICS_SDK=1` and the gripper backend is
linked for real. No vendor SDK is bundled in this repository - clone
yours from the URL above.

## Windows notes

The qbRobotics USB / RS-485 adapter uses FTDI internally. If
Windows does not enumerate the COM port automatically, install the
official FTDI VCP drivers from
https://ftdichip.com/drivers/vcp-drivers/ and check Device Manager
for the port name (Ports -> COM & LPT).

## What this repository does NOT do

The historical qbRobotics Doosan plugin (qbGrippers task editor module)
is intentionally NOT used here. Gripper commands originate from the
Windows PC running Leap_control via `qbmove_communications` and reach
the SoftClaw over USB / RS-485 directly. The Doosan controller stays
responsible for robot motion only (DRFL).

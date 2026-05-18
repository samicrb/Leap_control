# qb SoftClaw integration (PC-side, qbRobotics SDK)

The qb SoftClaw is controlled **directly from the Windows PC that runs
Leap_control**, via the official qbRobotics C/C++ API. The Doosan
controller is responsible for robot motion only.

## What this integration does NOT use

- No Doosan tool digital outputs (DO).
- No Doosan controller tool I/O for gripper commands.
- No qbGrippers "Write Signals" inside the Doosan Task Editor.
- No Doosan controller plugin / module.
- No Modbus path.

## SDK source

- Upstream: https://github.com/NMMI/qbAPI
- Header: `qbmove_communications.h`
- Key functions used:
  - `openRS485(comm_settings*, const char* port, int baud)`
  - `commPing(comm_settings*, int id)`
  - `commActivate(comm_settings*, int id, char activate)`
  - `commSetInputs(comm_settings*, int id, short int inputs[2])`
    - `inputs[0]` = motor reference position (encoder ticks)
    - `inputs[1]` = stiffness preset / deflection (encoder ticks)
  - `closeRS485(comm_settings*)`

## Build integration

1. Clone the SDK alongside the source:
   ```
   cd external
   git clone https://github.com/NMMI/qbAPI.git qbrobotics_sdk
   ```
   Or pass `-DQBROBOTICS_SDK_ROOT="C:/path/to/qbAPI"` to CMake.

2. Configure & build with Visual Studio:
   ```
   cmake -B out\build\x64-Debug -S . -A x64
   cmake --build out\build\x64-Debug
   ```

   When the SDK is detected, CMake reports
   `qbRobotics qbAPI found at <path> (real SoftClaw control enabled).`
   and defines `HAVE_QBROBOTICS_SDK=1`.

   When the SDK is missing, the build still succeeds and
   `QbSoftClawGripper` is compiled as a clearly-logged stub.

## Finding the COM port on Windows

1. Plug the qbRobotics USB / RS-485 adapter into the PC.
2. Open Device Manager.
3. Expand "Ports (COM & LPT)".
4. The adapter appears as `USB Serial Port (COMx)` (FTDI driver).
5. Note the `COMx` number and set it in `demo_config.ini`:
   ```
   gripper.port = COM3
   ```
6. If no port appears, install the FTDI VCP drivers from
   https://ftdichip.com/drivers/vcp-drivers/ and replug.

## Running

```
.\out\build\x64-Debug\doosan_gesture_demo.exe --config config\demo_config.ini
```

Expected boot lines on stdout/stderr:

```
INFO  Gripper:  enabled=true type='softclaw' backend='qb_sdk'
INFO  Gripper selection: type='softclaw' backend='qb_sdk' (port='COM3' device_id=1 required=false).
INFO  Gripper selection: PC-side qbRobotics SoftClaw via qbAPI.
INFO  Gripper: connecting to qb SoftClaw (port=COM3, device_id=1, baudrate=2000000).
INFO  Gripper: qb SoftClaw connected and activated (device id 1 on COM3).
```

If the SoftClaw is unreachable:

```
ERROR Gripper: openRS485 failed on port COM3 (check Device Manager...)
WARN  Gripper: SoftClaw initialization failed; [gripper].required=false
      -> falling back to NoopGripper.
```

## Configuration cheat sheet

| Key | Default | Description |
|-----|---------|-------------|
| `gripper.enabled`                  | `false`     | Master enable. `false` = NoopGripper, no I/O. |
| `gripper.type`                     | `softclaw`  | `softclaw` / `qb_softclaw` selects the SoftClaw backend. |
| `gripper.backend`                  | `qb_sdk`    | `qb_sdk` = PC-side qbAPI. Other values: `tool_io`, `none`. |
| `gripper.port`                     | `COM3`      | Windows COM port for the USB / RS-485 adapter. |
| `gripper.device_id`                | `1`         | qbRobotics device id on the bus. |
| `gripper.baudrate`                 | `2000000`   | qbAPI default on Windows. |
| `gripper.required`                 | `false`     | When `true`, init failure aborts startup. |
| `gripper.initialize_on_startup`    | `true`      | Open serial + ping + activate on app start. |
| `gripper.open_on_startup`          | `false`     | Optional one-shot OPEN after init. |
| `gripper.stop_on_exit`             | `true`      | Deactivate motor on app exit. |
| `gripper.open_position`            | `0`         | `commSetInputs` inputs[0] for OPEN. |
| `gripper.close_position`           | `19000`     | `commSetInputs` inputs[0] for CLOSE. |
| `gripper.open_deflection`          | `0`         | `commSetInputs` inputs[1] for OPEN. |
| `gripper.close_deflection`         | `15000`     | `commSetInputs` inputs[1] for CLOSE. |
| `gripper.min_command_period_ms`    | `30`        | Hard minimum between serial writes. |
| `gripper.command_deadband`         | `50`        | Skip duplicate / sub-threshold updates. |
| `gripper.command_timeout_ms`       | `100`       | Budget for read-back / ping operations. |

## Troubleshooting

- **No COM port in Device Manager.** Install FTDI VCP drivers, replug.
- **`commPing` returns -1.** Wrong `device_id`, SoftClaw unpowered, or
  another process holds the COM port. Close the qbRobotics GUI first.
- **`openRS485` fails on Windows but the port is visible.** Another
  process owns it. Close terminals / serial monitors.
- **The SoftClaw activates but does not move.** Verify the
  `open_position` / `close_position` are inside the safe range of your
  specific SoftClaw revision (qbRobotics ships per-device limits).
- **Build does NOT report "real SoftClaw control enabled."** The SDK
  was not detected. Clone NMMI/qbAPI into `external/qbrobotics_sdk` or
  pass `-DQBROBOTICS_SDK_ROOT=<path>` and reconfigure.

## What remains to test with the SoftClaw physically plugged

- COM port discovery on the production PC.
- `commPing` round-trip with the configured device id.
- Open / close encoder ticks per actual SoftClaw revision.
- Stiffness ramp behaviour vs configured deflection.
- Behaviour on serial cable disconnect mid-run.
- Behaviour on Windows sleep / USB re-enumeration.

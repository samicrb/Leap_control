#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace dgd {

// QbSoftClawGripper - PC-side control for the qbRobotics qb SoftClaw
// using the official qbRobotics C API (https://github.com/NMMI/qbAPI).
//
// IMPORTANT:
//   - The SoftClaw is connected to the Windows PC via the qbRobotics
//     USB / RS-485 adapter. Commands are issued by THIS process over a
//     COM port (e.g. COM3) at 2 000 000 baud (default qbAPI value).
//   - The Doosan controller is NOT involved in gripper control. No
//     tool digital outputs are used; no qbGrippers "Write Signals" in
//     the Doosan Task Editor are used. The Doosan controller is used
//     for robot motion (DRFL) only.
//
// SoftClaw command model (qbAPI commSetInputs):
//   inputs[0] = motor reference position    (encoder ticks)
//   inputs[1] = stiffness preset / deflection (encoder ticks)
//
// Build modes:
//   HAVE_QBROBOTICS_SDK=1 -> real qbAPI calls (openRS485 / commActivate
//                            / commSetInputs / commPing / closeRS485)
//   HAVE_QBROBOTICS_SDK=0 -> clear stub: log every intended call and
//                            return success so the rest of the demo
//                            can run. SoftClaw is then effectively
//                            disconnected and required=false fallback
//                            kicks in (see GripperFactory).
//
// Threading:
//   open() / close() are called only from the Application's tick loop,
//   AFTER the gesture layer has produced an impulse. Continuous
//   commands at 60 Hz are NOT supported - the backend rate-limits via
//   gripper_min_command_period_ms and gripper_command_deadband and
//   skips duplicate / sub-threshold updates.
class QbSoftClawGripper final : public IGripperController {
public:
    explicit QbSoftClawGripper(const Config& cfg);
    ~QbSoftClawGripper() override;

    // Identical to initialize() - kept for compatibility with the old
    // IGripperController API surface used by ToolIoGripperController.
    bool connect() override;
    void disconnect() override;
    bool isConnected() const override { return connected_.load(); }

    bool open() override;
    bool close() override;

    // Continuous setpoint helper used by gesture-aware callers that
    // want intermediate positions (0 = open_position, 100 = close_position
    // / close_deflection). Rate-limited and deadbanded like open/close.
    bool setClosurePercent(double percent);

    // Send a controlled stop / motor deactivate. Called from
    // Application::run() on shutdown when stop_on_exit = true.
    void stop() override;

    bool        initialize() override;
    bool        isAvailable() const override { return connected_.load(); }
    const char* name() const override { return "qb_softclaw"; }

    State       lastCommandedState() const override { return last_state_; }
    std::string lastError()          const override { return last_error_; }

private:
    bool sendInputs(int position, int deflection, const char* tag);
    bool rateLimit(int position, int deflection, double now_s);
    static double nowMonotonic();

    const Config& cfg_;

    // Opaque pImpl that owns the qbAPI handle (only when the SDK is
    // linked). Keeping the qbAPI types out of this header lets the
    // rest of the codebase build identically with or without the SDK.
    struct Impl;
    std::unique_ptr<Impl> p_;

    std::atomic<bool> connected_{false};
    State             last_state_ = State::Unknown;
    std::string       last_error_;

    // Rate-limit / deadband bookkeeping for the SoftClaw serial bus.
    double last_command_s_   = -1.0;
    int    last_position_    = -1;
    int    last_deflection_  = -1;
};

} // namespace dgd

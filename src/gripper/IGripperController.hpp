#pragma once

#include <string>

namespace dgd {

// Abstract gripper interface. Impulse-style API on purpose:
// Application sends a single open() / close() when the gesture layer
// produces an impulse event. Continuous-command behaviour must not be
// supported by any implementation (Rule 7 of the context).
//
// Optional virtuals (initialize / stop / name / isAvailable) have
// default no-op implementations so legacy backends like
// ToolIoGripperController keep building unchanged. Vendor backends
// (e.g. qbRobotics SoftClaw via qbAPI) override them.
class IGripperController {
public:
    virtual ~IGripperController() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    virtual bool open()  = 0;
    virtual bool close() = 0;

    // Last physical state we commanded (not necessarily measured).
    enum class State { Unknown, Open, Closed };
    virtual State lastCommandedState() const = 0;

    virtual std::string lastError() const = 0;

    // --- Optional vendor-friendly API (default = forwards / no-op) -----
    // initialize() is the explicit lifecycle entry point used by
    // GripperFactory + Application::initialise. Default forwards to
    // connect() so existing backends behave unchanged.
    virtual bool initialize() { return connect(); }

    // Send a controlled stop / release. Default = no-op (acceptable for
    // two-state DO grippers; the qbRobotics SoftClaw backend uses this
    // to deactivate the motor on shutdown).
    virtual void stop() {}

    // Human-readable backend tag for logs and CSV diagnostics.
    virtual const char* name() const { return "gripper"; }

    // True iff the underlying hardware is currently reachable. Defaults
    // to isConnected().
    virtual bool isAvailable() const { return isConnected(); }
};

} // namespace dgd

#pragma once

#include <string>

namespace dgd {

// Abstract gripper interface. Impulse-style API on purpose:
// Application sends a single open() / close() when the gesture layer
// produces an impulse event. Continuous-command behaviour must not be
// supported by any implementation (Rule 7 of the context).
//
// The optional methods (initialize / setClosure / stopOrHold /
// isAvailable) have default no-op implementations so existing two-state
// backends (e.g. tool-DO) remain ABI-stable. Vendor-specific backends
// (qbRobotics SoftClaw) override them with real behaviour.
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

    // --- Optional vendor-friendly API (default = no-op) ----------------
    // Alias for connect(); separated so callers can express intent.
    virtual bool initialize() { return connect(); }
    // Continuous closure percentage in [0..100]. 0 = fully open,
    // 100 = fully closed. Default = unsupported.
    virtual bool setClosure(double /*percent*/) { return false; }
    // Stop motion / hold current position without releasing. Default = no-op.
    virtual void stopOrHold() {}
    // True if the backend believes the hardware is reachable. Defaults to
    // isConnected() so existing backends keep working.
    virtual bool isAvailable() const { return isConnected(); }
};

} // namespace dgd

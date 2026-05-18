#pragma once

#include "gripper/IGripperController.hpp"

namespace dgd {

// NoopGripper - silent no-op backend.
//
// Used when the gripper section is disabled in demo_config.ini, or when
// an optional vendor backend (e.g. qbRobotics SoftClaw via qbAPI) is
// the requested backend but the hardware is not detected. All methods
// return success but perform no I/O, so the rest of the Application
// can call open() / close() unconditionally on gesture impulses.
class NoopGripper final : public IGripperController {
public:
    bool connect() override;
    void disconnect() override {}
    bool isConnected() const override { return false; }

    bool open() override  { last_state_ = State::Open;   return true; }
    bool close() override { last_state_ = State::Closed; return true; }

    State       lastCommandedState() const override { return last_state_; }
    std::string lastError()          const override { return ""; }

    bool        initialize() override { return connect(); }
    void        stop() override {}
    const char* name() const override { return "noop"; }
    bool        isAvailable() const override { return false; }

private:
    State last_state_ = State::Unknown;
};

} // namespace dgd

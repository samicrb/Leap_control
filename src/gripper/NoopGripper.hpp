#pragma once

#include "gripper/IGripperController.hpp"

namespace dgd {

// NoopGripper - silent no-op backend.
//
// Used whenever [gripper] enabled = false (the default). All methods
// return success but perform no I/O, no logging beyond a single
// connect() info line, and never touch the controller. This lets the
// rest of the Application call open() / close() unconditionally on
// gesture impulses without growing extra null-checks.
class NoopGripper final : public IGripperController {
public:
    bool connect() override;
    void disconnect() override {}
    bool isConnected() const override { return false; }

    bool open() override  { last_state_ = State::Open;   return true; }
    bool close() override { last_state_ = State::Closed; return true; }

    State       lastCommandedState() const override { return last_state_; }
    std::string lastError()          const override { return ""; }

    // Optional API: tolerated, intentionally no-op.
    bool setClosure(double /*percent*/) override { return true; }
    void stopOrHold() override {}
    bool isAvailable() const override { return false; }

private:
    State last_state_ = State::Unknown;
};

} // namespace dgd

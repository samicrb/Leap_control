#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"

#include <atomic>
#include <string>

namespace dgd {

// qbRobotics SoftHand Industry - PLACEHOLDER backend.
//
// This implementation does NOT link the qbRobotics SDK. It exists so
// the project compiles and runs in the field with [gripper] enabled =
// true while the vendor integration is being built. Every call logs
// the intended action; no socket / serial / API call is performed.
//
// Replace the bodies of open() / close() / setClosure() with the
// qbRobotics UDP/API calls when the SDK is available. The interface
// shape (impulse-style open/close + optional continuous setClosure)
// matches the eventual integration path so callers do not need to
// change.
//
// IMPORTANT: gripper commands must remain event-based. NEVER call this
// in the 60 Hz motion loop; Application invokes open() / close() only
// on gesture impulses.
class QbSoftHandIndustryGripper final : public IGripperController {
public:
    explicit QbSoftHandIndustryGripper(const Config& cfg);

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override { return connected_.load(); }

    bool open() override;
    bool close() override;

    State       lastCommandedState() const override { return last_state_; }
    std::string lastError()          const override { return last_error_; }

    // Continuous closure 0..100 (% of close_position). Stub-logs only.
    bool setClosure(double percent) override;
    void stopOrHold() override;
    bool isAvailable() const override { return connected_.load(); }

private:
    const Config&     cfg_;
    std::atomic<bool> connected_{false};
    State             last_state_ = State::Unknown;
    std::string       last_error_;
};

} // namespace dgd

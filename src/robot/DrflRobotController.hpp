#pragma once

#include "robot/IRobotController.hpp"
#include "config/Config.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace dgd {

// DRFL 1.33.2 adapter. This class is the ONLY translation unit that
// should include DRFL headers.
//
// When built without DRFL the adapter behaves as a cooperative simulator:
//   - reports connected after ~100 ms,
//   - integrates the commanded twist into a fake pose (so the workspace
//     guard, state machine and UI can be fully exercised),
//   - logs every call that would be sent to DRFL.
//
// DRFL-INTEGRATION: every call site that must talk to the real robot is
// tagged with the comment "// DRFL:". Wire those up on the event PC.
class DrflRobotController final : public IRobotController {
public:
    explicit DrflRobotController(const Config& cfg);
    ~DrflRobotController() override;

    bool connect(const std::string& ip, int port, double timeout_s) override;
    void disconnect() override;
    bool isConnected() const override { return connected_.load(); }

    bool engage() override;
    void disengage() override;
    bool moveHome(const RobotPose& safe) override;
    bool setTcp(const std::string& name) override;
    bool setToolWeight(const std::string& name) override;

    bool sendCartesianVelocity(const std::array<double, 6>& twist) override;
    void stopMotion() override;
    void emergencyStop() override;
    bool sendCartesianMicroMove(const RobotPose& target,
                                double lin_vel, double ang_vel,
                                double lin_acc, double ang_acc,
                                double blending_radius_mm = 0.0) override;
    bool getCurrentPose(RobotPose& out) override;

    std::string lastError() const override { return last_error_; }

private:
    const Config& cfg_;

    struct Impl;
    std::unique_ptr<Impl> p_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> engaged_{false};
    std::string last_error_;

    // Fake / last-known pose (used only in DRFL-absent or dry-run modes).
    mutable std::mutex pose_mx_;
    RobotPose current_pose_;
    double    last_twist_time_s_ = 0.0;
    std::array<double, 6> last_twist_ {0,0,0,0,0,0};
    // Tracks whether the last speedl we issued was at zero so we can
    // emit a single zero command on the falling edge (active->idle) and
    // then stay quiet, rather than spamming speedl(0) at 60 Hz.
    bool      last_was_zero_ = true;
};

} // namespace dgd

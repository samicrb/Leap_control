#pragma once

#include "motion/IMotionBackend.hpp"
#include "robot/RobotPose.hpp"

#include <optional>
#include <string>

namespace dgd {

class IRobotController;
struct Config;

// ServolMotionBackend - continuous Doosan SERVO-L target update.
//
// Each tick the backend:
//   1. Recomputes the desired absolute target from the SM-issued
//      velocity (recovered hand displacement * micro_hand_to_robot_ratio).
//   2. Applies a Euclidean vector-norm cap on the delta from the current
//      internal target. NEVER clamps per-axis (the historical 38 mm bug).
//   3. Optionally applies the post-recovery soft cap from the
//      tracking-loss tolerance machinery.
//   4. If at least min_period_s has elapsed AND the step exceeds the
//      arrival band, emits servol(target, vel, acc, time).
//
// On brief tracking loss the backend FREEZES its internal target so the
// robot holds (servol continues to receive the same pose every tick at
// the configured rate). On hard loss / SM fault / passive state, it
// skips emission and resets on the next active entry.
//
// No blending, no backlog guard, no amovel-specific machinery.
class ServolMotionBackend final : public IMotionBackend {
public:
    const char* name() const override { return "servol"; }
    MotionBackendKind kind() const override { return MotionBackendKind::Servol; }

    void attach(const Config& cfg, IRobotController& robot) override;
    void onActiveEntry(const RobotPose& seed_pose, double now_s) override;
    void onActiveExit(double now_s) override;
    void onReanchor(const RobotPose& anchor_pose, double now_s) override;
    MotionTickResult onTick(const MotionTickContext& ctx) override;
    std::optional<RobotPose> currentTarget() const override {
        return target_valid_ ? std::optional<RobotPose>(target_) : std::nullopt;
    }

private:
    const Config*    cfg_   = nullptr;
    IRobotController* robot_ = nullptr;

    // Internal absolute target pose. Updated each tick from the desired
    // target with the vector-norm cap; emitted to the robot at most
    // every command_rate_hz.
    RobotPose target_      {};
    bool      target_valid_ = false;
    double    last_command_s_ = -1.0;  // wall time of last servol emit
    bool      logged_init_   = false;
};

} // namespace dgd

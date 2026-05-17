#pragma once

#include "config/Config.hpp"
#include "robot/RobotPose.hpp"

#include <array>

namespace dgd {

// Enforces the demo workspace envelope. The guard does NOT plan a
// stopping trajectory; it just clamps the commanded twist so the next
// integration step stays inside the box. Application is responsible for
// calling it before forwarding a twist to the robot controller.
class WorkspaceGuard {
public:
    WorkspaceGuard(const Config& cfg, const RobotPose& safe) : cfg_(cfg), safe_(safe) {}

    // Clamp twist (vx,vy,vz,wx,wy,wz) given current pose. Returns true
    // if the command touched a boundary (for UI / logging).
    bool clamp(const RobotPose& current, std::array<double, 6>& twist, double dt_s) const;

    // Apply ONLY the hard speed / acceleration caps to the twist. Does
    // not require a pose, so safe to call during streaming
    // (PositionControl / OrientationControl) where reading the pose
    // would interleave a non-motion DRFL call (CONTROL_CHECK_CURRENT_TASK_POSITION)
    // between two speedl()s and trip alarm 5.7056.
    void clampSpeed(std::array<double, 6>& twist) const;

private:
    const Config& cfg_;
    RobotPose safe_;
};

} // namespace dgd

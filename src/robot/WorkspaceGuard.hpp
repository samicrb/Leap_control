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

private:
    const Config& cfg_;
    RobotPose safe_;
};

} // namespace dgd

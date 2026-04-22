#pragma once

#include <array>

namespace dgd {

// Doosan TCP pose in base frame: X/Y/Z in mm, Rx/Ry/Rz in degrees.
struct RobotPose {
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;
    double rx = 0.0;
    double ry = 180.0;
    double rz = 0.0;

    std::array<double, 6> toArray() const { return {x, y, z, rx, ry, rz}; }
    static RobotPose fromArray(const std::array<double, 6>& a) {
        return {a[0], a[1], a[2], a[3], a[4], a[5]};
    }
};

} // namespace dgd

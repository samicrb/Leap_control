#pragma once

#include <array>

namespace dgd {

// Doosan TCP pose in BASE frame.
//
// Units:
//   x, y, z       - millimetres
//   rx, ry, rz    - degrees
//
// Rotation convention (Doosan V3.5 controller):
//   Despite their names, (rx, ry, rz) here are NOT extrinsic XYZ Euler
//   angles around the BASE X/Y/Z axes. They are the Doosan convention
//   (W, P, R) i.e. ZYZ' INTRINSIC Euler:
//       rx == W : rotation about BASE  Z   (first)
//       ry == P : rotation about ROT-1 Y'  (second, around the new Y)
//       rz == R : rotation about ROT-2 Z'' (third,  around the new Z)
//   This matches what the teach pendant displays as "posx [x,y,z,W,P,R]"
//   and what DRFL's movel/movejx/posx APIs consume.
//
// The fields are kept named rx/ry/rz for historical reasons - renaming
// them would touch every gesture-mapping site. Treat them as W/P/R
// whenever you reason about pose semantics.
struct RobotPose {
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;
    double rx = 0.0;       // == W (Z rotation,  Doosan ZYZ')
    double ry = 180.0;     // == P (Y' rotation, Doosan ZYZ')
    double rz = 0.0;       // == R (Z'' rotation, Doosan ZYZ')

    std::array<double, 6> toArray() const { return {x, y, z, rx, ry, rz}; }
    static RobotPose fromArray(const std::array<double, 6>& a) {
        return {a[0], a[1], a[2], a[3], a[4], a[5]};
    }
};

} // namespace dgd

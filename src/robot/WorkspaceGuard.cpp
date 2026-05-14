#include "robot/WorkspaceGuard.hpp"
#include "util/MathUtils.hpp"

#include <cmath>

namespace dgd {

namespace {
// If the pose is already past a limit, we still allow motion that pulls
// it back toward the envelope.
double clampAxisVel(double cur, double vel, double lo, double hi, double dt) {
    double nxt = cur + vel * dt;
    if (nxt < lo && vel < 0.0) return 0.0;
    if (nxt > hi && vel > 0.0) return 0.0;
    return vel;
}
} // namespace

bool WorkspaceGuard::clamp(const RobotPose& p, std::array<double, 6>& t, double dt) const {
    bool touched = false;

    // Envelope (position cube + orientation cone) is optional.
    if (cfg_.ws_enabled) {
        double vx = clampAxisVel(p.x,  t[0], cfg_.ws_x_min, cfg_.ws_x_max, dt);
        double vy = clampAxisVel(p.y,  t[1], cfg_.ws_y_min, cfg_.ws_y_max, dt);
        double vz = clampAxisVel(p.z,  t[2], cfg_.ws_z_min, cfg_.ws_z_max, dt);
        if (vx != t[0] || vy != t[1] || vz != t[2]) touched = true;
        t[0] = vx; t[1] = vy; t[2] = vz;

        double rx_lo = safe_.rx - cfg_.ws_rx_range, rx_hi = safe_.rx + cfg_.ws_rx_range;
        double ry_lo = safe_.ry - cfg_.ws_ry_range, ry_hi = safe_.ry + cfg_.ws_ry_range;
        double rz_lo = safe_.rz - cfg_.ws_rz_range, rz_hi = safe_.rz + cfg_.ws_rz_range;

        double wx = clampAxisVel(p.rx, t[3], rx_lo, rx_hi, dt);
        double wy = clampAxisVel(p.ry, t[4], ry_lo, ry_hi, dt);
        double wz = clampAxisVel(p.rz, t[5], rz_lo, rz_hi, dt);
        if (wx != t[3] || wy != t[4] || wz != t[5]) touched = true;
        t[3] = wx; t[4] = wy; t[5] = wz;
    }

    // Hard speed cap is ALWAYS applied. Qualify to avoid clashing with the
    // member name.
    t[0] = ::dgd::clamp(t[0], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[1] = ::dgd::clamp(t[1], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[2] = ::dgd::clamp(t[2], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[3] = ::dgd::clamp(t[3], -cfg_.max_ang_speed, cfg_.max_ang_speed);
    t[4] = ::dgd::clamp(t[4], -cfg_.max_ang_speed, cfg_.max_ang_speed);
    t[5] = ::dgd::clamp(t[5], -cfg_.max_ang_speed, cfg_.max_ang_speed);
    return touched;
}

void WorkspaceGuard::clampSpeed(std::array<double, 6>& t) const {
    t[0] = ::dgd::clamp(t[0], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[1] = ::dgd::clamp(t[1], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[2] = ::dgd::clamp(t[2], -cfg_.max_lin_speed, cfg_.max_lin_speed);
    t[3] = ::dgd::clamp(t[3], -cfg_.max_ang_speed, cfg_.max_ang_speed);
    t[4] = ::dgd::clamp(t[4], -cfg_.max_ang_speed, cfg_.max_ang_speed);
    t[5] = ::dgd::clamp(t[5], -cfg_.max_ang_speed, cfg_.max_ang_speed);
}

} // namespace dgd

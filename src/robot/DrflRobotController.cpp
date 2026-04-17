// DrflRobotController.cpp
//
// DRFL 1.33.2 integration shim.
//
// The Doosan V3.5 controller exposes a realtime external control mode
// reachable via DRFL. For a gesture-driven demo the natural primitives
// are:
//   - movej / movel  : blocking planned motions (used by moveHome)
//   - speedl         : stream an instantaneous Cartesian velocity twist
//   - stop           : safe emergency halt
//
// This adapter assumes those names. If your installed DRFL header uses
// CamelCase wrappers (e.g. DRFL::movel vs movel) just update the lines
// tagged "// DRFL:".
//
// Build modes:
//   HAVE_DRFL=1 -> real DRFL calls
//   HAVE_DRFL=0 -> cooperative simulator (default on dev machines)
//
// This file is deliberately the only place where DRFL appears.

#include "robot/DrflRobotController.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#if defined(HAVE_DRFL) && HAVE_DRFL
  #include <DRFLEx.h>
  using namespace DRAFramework;
#endif

namespace dgd {

namespace {
double nowSeconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}
} // namespace

struct DrflRobotController::Impl {
#if defined(HAVE_DRFL) && HAVE_DRFL
    CDRFLEx drfl;
#endif
};

DrflRobotController::DrflRobotController(const Config& cfg)
    : cfg_(cfg), p_(std::make_unique<Impl>()) {
    current_pose_ = { cfg.safe_x, cfg.safe_y, cfg.safe_z,
                      cfg.safe_rx, cfg.safe_ry, cfg.safe_rz };
}

DrflRobotController::~DrflRobotController() { disconnect(); }

bool DrflRobotController::connect(const std::string& ip, int port, double /*timeout_s*/) {
    if (cfg_.dryrun_robot) {
        LOG_I("Robot DRY-RUN: skipping real connection to %s:%d", ip.c_str(), port);
        connected_.store(true);
        return true;
    }

#if defined(HAVE_DRFL) && HAVE_DRFL
    // DRFL: connect to controller.
    if (!p_->drfl.open_connection(ip.c_str(), static_cast<unsigned int>(port))) {
        last_error_ = "DRFL open_connection failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
    // DRFL: set robot mode to manual/auto depending on teaching.
    p_->drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    p_->drfl.set_robot_system(ROBOT_SYSTEM_REAL);
    connected_.store(true);
    LOG_I("DRFL connected to %s:%d", ip.c_str(), port);
    return true;
#else
    // Simulator: pretend the connection succeeded after a short delay.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    connected_.store(true);
    LOG_W("DRFL not built in - using SIMULATOR (connected=%s:%d).", ip.c_str(), port);
    return true;
#endif
}

void DrflRobotController::disconnect() {
    if (!connected_.load()) return;
    disengage();
#if defined(HAVE_DRFL) && HAVE_DRFL
    p_->drfl.close_connection(); // DRFL:
#endif
    connected_.store(false);
    LOG_I("Robot disconnected.");
}

bool DrflRobotController::engage() {
    if (!connected_.load()) return false;
#if defined(HAVE_DRFL) && HAVE_DRFL
    // DRFL: set safety/servo on.
    if (!p_->drfl.set_robot_control(CONTROL_SERVO_ON)) {
        last_error_ = "set_robot_control(SERVO_ON) failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
#endif
    engaged_.store(true);
    LOG_I("Robot engaged (servo ON).");
    return true;
}

void DrflRobotController::disengage() {
    if (!engaged_.load()) return;
    stopMotion();
#if defined(HAVE_DRFL) && HAVE_DRFL
    p_->drfl.set_robot_control(CONTROL_SERVO_OFF); // DRFL:
#endif
    engaged_.store(false);
    LOG_I("Robot disengaged (servo OFF).");
}

bool DrflRobotController::moveHome(const RobotPose& safe) {
    if (!connected_.load()) return false;
    LOG_I("Robot moveHome -> (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)",
          safe.x, safe.y, safe.z, safe.rx, safe.ry, safe.rz);

#if defined(HAVE_DRFL) && HAVE_DRFL
    // DRFL: planned movel to the safe pose at a conservative speed.
    float target[6] = { (float)safe.x, (float)safe.y, (float)safe.z,
                        (float)safe.rx, (float)safe.ry, (float)safe.rz };
    float vel[2]   = { 50.0f, 10.0f };   // mm/s, deg/s
    float accel[2] = { 200.0f, 60.0f };
    // MOVE_REFERENCE_BASE, blocking.
    if (!p_->drfl.movel(target, vel, accel, 0.0f, MOVE_MODE_ABSOLUTE,
                        MOVE_REFERENCE_BASE, 0.0f, BLENDING_SPEED_TYPE_DUPLICATE)) {
        last_error_ = "DRFL movel(home) failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
#else
    // Simulator.
    std::lock_guard<std::mutex> lock(pose_mx_);
    current_pose_ = safe;
#endif
    return true;
}

bool DrflRobotController::sendCartesianVelocity(const std::array<double, 6>& twist) {
    if (!connected_.load() || !engaged_.load()) return false;

    {
        std::lock_guard<std::mutex> lock(pose_mx_);
        last_twist_ = twist;

        // Cooperative simulator: integrate the twist so the state
        // machine / UI have something to display during dev.
        double now = nowSeconds();
        double dt  = last_twist_time_s_ > 0.0 ? (now - last_twist_time_s_) : 0.01;
        last_twist_time_s_ = now;
        current_pose_.x  += twist[0] * dt;
        current_pose_.y  += twist[1] * dt;
        current_pose_.z  += twist[2] * dt;
        current_pose_.rx += twist[3] * dt;
        current_pose_.ry += twist[4] * dt;
        current_pose_.rz += twist[5] * dt;
    }

    if (cfg_.dryrun_robot) {
        return true;
    }

#if defined(HAVE_DRFL) && HAVE_DRFL
    // DRFL: speedl streams an instantaneous Cartesian velocity.
    // Argument: {vx, vy, vz, wx, wy, wz}, accel cap, duration.
    float vel[6] = { (float)twist[0], (float)twist[1], (float)twist[2],
                     (float)twist[3], (float)twist[4], (float)twist[5] };
    float accel[2] = { (float)cfg_.max_lin_accel, (float)cfg_.max_ang_accel };
    if (!p_->drfl.speedl(vel, accel, 0.1f)) {
        last_error_ = "DRFL speedl failed";
        return false;
    }
#endif
    return true;
}

void DrflRobotController::stopMotion() {
    std::lock_guard<std::mutex> lock(pose_mx_);
    last_twist_ = {0,0,0,0,0,0};
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (connected_.load() && engaged_.load()) {
        // DRFL: a zero speedl halts streaming; a proper stop is cleaner.
        p_->drfl.stop(STOP_TYPE_QUICK);
    }
#endif
}

bool DrflRobotController::getCurrentPose(RobotPose& out) {
    if (!connected_.load()) return false;
#if defined(HAVE_DRFL) && HAVE_DRFL
    // DRFL: latest measured TCP pose.
    LPROBOT_POSE p = p_->drfl.get_current_posx();
    if (!p) return false;
    out.x  = p->_fPosition[0];
    out.y  = p->_fPosition[1];
    out.z  = p->_fPosition[2];
    out.rx = p->_fPosition[3];
    out.ry = p->_fPosition[4];
    out.rz = p->_fPosition[5];
    {
        std::lock_guard<std::mutex> lock(pose_mx_);
        current_pose_ = out; // keep simulator in sync
    }
    return true;
#else
    std::lock_guard<std::mutex> lock(pose_mx_);
    out = current_pose_;
    return true;
#endif
}

} // namespace dgd

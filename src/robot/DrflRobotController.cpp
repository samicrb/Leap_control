// DrflRobotController.cpp
//
// DRFL 1.33.x integration shim (validated against API-DRFL 1.33.3).
//
// Canonical DRFL bring-up sequence used here:
//   1. open_connection(ip, port)
//   2. setup_monitoring_version(1)
//   3. register monitoring callbacks (state, access control, log alarm)
//   4. manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)
//   5. wait for MONITORING_ACCESS_CONTROL_GRANT (timeout cfg_.connect_timeout_s)
//   6. if state == STATE_SAFE_OFF: set_robot_control(CONTROL_SERVO_ON)
//   7. wait for state == STATE_STANDBY
//   8. set_robot_mode(ROBOT_MODE_AUTONOMOUS)
//   9. set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP)
//  10. movel / speedl / stop
//
// DRFL exposes plain-C callback function pointers (no userdata channel).
// We therefore stash the bits of state callbacks need in file-scope atomics.
// Only one DrflRobotController is constructed per process, which makes
// this safe.
//
// Build modes:
//   HAVE_DRFL=1 -> real DRFL calls
//   HAVE_DRFL=0 -> cooperative simulator (default on dev machines)

#include "robot/DrflRobotController.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#if defined(HAVE_DRFL) && HAVE_DRFL
  // DRFL 1.33.x headers ship with several C4244 conversions on MSVC /W4.
  // Localise the suppression so the rest of our code keeps building clean.
  #ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4244)
  #endif
  #include <DRFLEx.h>
  #ifdef _MSC_VER
    #pragma warning(pop)
  #endif
  using namespace DRAFramework;
#endif

namespace dgd {

namespace {
double nowSeconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

#if defined(HAVE_DRFL) && HAVE_DRFL
// DRFL monitoring callbacks. File-scope state; single-instance assumption.
std::atomic<int>  g_robot_state{static_cast<int>(STATE_NOT_READY)};
std::atomic<int>  g_access_grant{0};   // 0 pending, 1 granted, -1 lost/denied
std::atomic<bool> g_alarm_seen{false};

void onMonitoringState(const ROBOT_STATE eState) {
    g_robot_state.store(static_cast<int>(eState));
}
void onMonitoringAccessControl(const MONITORING_ACCESS_CONTROL eTrans) {
    switch (eTrans) {
        case MONITORING_ACCESS_CONTROL_GRANT:
            g_access_grant.store(1);
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
            g_access_grant.store(-1);
            break;
        default:
            break;
    }
}
void onLogAlarm(LPLOG_ALARM pLogAlarm) {
    g_alarm_seen.store(true);
    if (!pLogAlarm) return;
    // Surface the canonical Doosan alarm tuple. Cross-reference index in
    // the Doosan alarm manual to identify the trip cause.
    LOG_E("DRFL alarm: level=%d group=%d index=%d params=[%s | %s | %s]",
          pLogAlarm->_iLevel, pLogAlarm->_iGroup, pLogAlarm->_iIndex,
          pLogAlarm->_szParam[0], pLogAlarm->_szParam[1], pLogAlarm->_szParam[2]);
}

// Block until the predicate becomes true or `timeout_s` elapses.
template <typename Pred>
bool waitFor(Pred pred, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return pred();
}
#endif
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
    // 1. Open the TCP link to the controller.
    if (!p_->drfl.open_connection(ip.c_str(), static_cast<unsigned int>(port))) {
        last_error_ = "DRFL open_connection failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }

    // 2. Subscribe to the monitoring stream and register callbacks BEFORE
    //    we request control authority so the GRANT event isn't missed.
    g_robot_state.store(static_cast<int>(STATE_NOT_READY));
    g_access_grant.store(0);
    g_alarm_seen.store(false);
    p_->drfl.setup_monitoring_version(1);
    p_->drfl.set_on_monitoring_state(onMonitoringState);
    p_->drfl.set_on_monitoring_access_control(onMonitoringAccessControl);
    p_->drfl.set_on_log_alarm(onLogAlarm);

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
    // 3. Request authority over the controller. FORCE_REQUEST is the demo-
    //    friendly variant: it pre-empts whoever currently holds control
    //    (typically the teach pendant) without asking for confirmation.
    g_access_grant.store(0);
    if (!p_->drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) {
        last_error_ = "manage_access_control(FORCE_REQUEST) failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }

    const double timeout = cfg_.connect_timeout_s > 0.0 ? cfg_.connect_timeout_s : 10.0;
    if (!waitFor([]{ return g_access_grant.load() == 1; }, timeout)) {
        last_error_ = "access control not granted within timeout";
        LOG_E("%s (last state=%d, grant=%d)",
              last_error_.c_str(), g_robot_state.load(), g_access_grant.load());
        return false;
    }
    LOG_I("DRFL access control GRANTED.");

    // 4. If the robot is in SAFE_OFF (servo off), turn the servos on.
    if (g_robot_state.load() == static_cast<int>(STATE_SAFE_OFF)) {
        if (!p_->drfl.set_robot_control(CONTROL_SERVO_ON)) {
            last_error_ = "set_robot_control(SERVO_ON) failed";
            LOG_E("%s", last_error_.c_str());
            return false;
        }
    }

    // 5. Wait for STATE_STANDBY before any motion command.
    if (!waitFor([]{ return g_robot_state.load() == static_cast<int>(STATE_STANDBY); },
                 timeout)) {
        last_error_ = "robot did not reach STATE_STANDBY";
        LOG_E("%s (last state=%d)", last_error_.c_str(), g_robot_state.load());
        return false;
    }

    // 6. Autonomous mode (no teach-pendant gating of subsequent motions).
    p_->drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    // 7. Force the safety mode into AUTONOMOUS. Some controllers boot in
    //    SAFETY_MODE_MANUAL (collaborative speed-limited) and trip a
    //    SAFE_STOP as soon as a movel exceeds the manual-mode speed cap.
    //    Pinning SAFETY_MODE_AUTONOMOUS here removes that footgun.
    if (!p_->drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP)) {
        last_error_ = "set_safety_mode(AUTONOMOUS, STOP) failed";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
#endif
    engaged_.store(true);
    LOG_I("Robot engaged (authority + servo ON + STANDBY + safety=AUTONOMOUS).");
    return true;
}

void DrflRobotController::disengage() {
    if (!engaged_.load()) return;
    stopMotion();
#if defined(HAVE_DRFL) && HAVE_DRFL
    // Note: DRFL 1.33.x does not consistently expose a RELEASE enumerant
    // on MANAGE_ACCESS_CONTROL. close_connection() (invoked from
    // disconnect()) already frees the authority when the socket drops, so
    // an explicit release call is not required for a clean shutdown.
#endif
    engaged_.store(false);
    LOG_I("Robot disengaged (motion stopped, authority will release on disconnect).");
}

bool DrflRobotController::moveHome(const RobotPose& safe) {
    if (!connected_.load()) return false;
    LOG_I("Robot moveHome -> (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)",
          safe.x, safe.y, safe.z, safe.rx, safe.ry, safe.rz);

#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "DRFL movel(home): no access control authority";
        LOG_E("%s (grant=%d, state=%d)",
              last_error_.c_str(), g_access_grant.load(), g_robot_state.load());
        return false;
    }
    // DRFL: planned movel to the safe pose. Profile is intentionally very
    // gentle so the controller's joint-speed / collision supervisions are
    // unlikely to trip on the initial approach. If a SAFE_STOP still fires
    // at these speeds, the cause is not velocity-related (workspace zone,
    // collision sensitivity, singularity) - set robot.skip_move_home=true
    // and inspect the pendant.
    float target[6] = { (float)safe.x, (float)safe.y, (float)safe.z,
                        (float)safe.rx, (float)safe.ry, (float)safe.rz };
    float vel[2]   = { 20.0f, 5.0f };    // mm/s, deg/s
    float accel[2] = { 50.0f, 20.0f };
    // MOVE_REFERENCE_BASE, blocking.
    if (!p_->drfl.movel(target, vel, accel, 0.0f, MOVE_MODE_ABSOLUTE,
                        MOVE_REFERENCE_BASE, 0.0f, BLENDING_SPEED_TYPE_DUPLICATE)) {
        last_error_ = "DRFL movel(home) failed (state="
                    + std::to_string(g_robot_state.load()) + ")";
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
    // DRFL: latest measured TCP pose. get_current_posx() returns the task-
    // space pose (LPROBOT_TASK_POSE), whose payload lives in _fTargetPos.
    LPROBOT_TASK_POSE p = p_->drfl.get_current_posx();
    if (!p) return false;
    out.x  = p->_fTargetPos[0];
    out.y  = p->_fTargetPos[1];
    out.z  = p->_fTargetPos[2];
    out.rx = p->_fTargetPos[3];
    out.ry = p->_fTargetPos[4];
    out.rz = p->_fTargetPos[5];
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

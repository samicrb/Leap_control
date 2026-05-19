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

    // 4. Auto-clear latched safety stops left over from a previous run.
    //    A SAFE_STOP / SAFE_OFF from a prior session stays latched on
    //    the controller and looks identical to a "mastering lost" state:
    //    servo_on below would silently refuse without this.
    //    The reset calls are no-ops when no stop is latched.
    if (cfg_.auto_reset_safety) {
        const int s0 = g_robot_state.load();
        if (s0 == static_cast<int>(STATE_SAFE_STOP) ||
            s0 == static_cast<int>(STATE_SAFE_STOP2)) {
            LOG_W("Robot latched in SAFE_STOP at startup; auto-resetting.");
            p_->drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (s0 == static_cast<int>(STATE_SAFE_OFF) ||
            s0 == static_cast<int>(STATE_SAFE_OFF2)) {
            LOG_W("Robot latched in SAFE_OFF at startup; auto-resetting.");
            p_->drfl.set_robot_control(CONTROL_RESET_SAFET_OFF);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (s0 == static_cast<int>(STATE_RECOVERY)) {
            // Real mastering loss / recovery-needed state: DRFL cannot
            // bypass this. Log loudly and let the user fix it on the
            // pendant. We continue and let the STANDBY wait time out so
            // the operator sees the failure with full context.
            LOG_E("Robot in RECOVERY state (likely mastering lost). "
                  "Bypass via DRFL is NOT possible - restore from teach "
                  "pendant: Setting -> Robot -> Mastering -> 'Use existing "
                  "mastering data' or run Auto Mastering.");
        }
    }

    // 5. If the robot is in SAFE_OFF (servo off), turn the servos on.
    if (g_robot_state.load() == static_cast<int>(STATE_SAFE_OFF) ||
        g_robot_state.load() == static_cast<int>(STATE_SAFE_OFF2)) {
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

    // 8. Collision-detection sensitivity. The Doosan A-series cobots ship
    //    with collision detection on by default; it compares measured
    //    joint torques against an inverse-dynamics prediction. If the
    //    payload / TCP declared on the controller doesn't match the
    //    actual mount, even gentle motion produces "unexpected" torques
    //    and trips a SAFE_OFF mid-trajectory. Override here so bring-up
    //    isn't blocked. 0.0 == disabled, 100.0 == most sensitive.
    p_->drfl.change_collision_sensitivity(
        static_cast<float>(cfg_.collision_sensitivity));
    LOG_I("Collision sensitivity set to %d (0 = disabled).",
          cfg_.collision_sensitivity);

    // 9. Singularity handling. Default is STOP (1) which stops the motion
    //    on entering a singular region - that's what trips alarm 3205
    //    followed by SAFE_OFF (7056) on the safe-pose movel. AVOID (0)
    //    makes the controller blend through automatically; VEL (2)
    //    reduces velocity in the region.
    //    Doosan SINGULARITY_AVOIDANCE enum: AVOID=0, STOP=1, VEL=2.
    const bool sing_ok = p_->drfl.set_singularity_handling(
        static_cast<SINGULARITY_AVOIDANCE>(cfg_.singularity_handling));
    LOG_I("Singularity handling set to %d (0=AVOID, 1=STOP, 2=VEL) ok=%d.",
          cfg_.singularity_handling, sing_ok ? 1 : 0);
#endif
    engaged_.store(true);
    LOG_I("Robot engaged (authority + servo ON + STANDBY + safety=AUTONOMOUS).");
    return true;
}

void DrflRobotController::disengage() {
    if (!engaged_.load()) return;
    // Normal cleanup path: soft stop only. Do NOT call emergencyStop()
    // here - shutdown / disconnect is not a critical fault and
    // STOP_TYPE_QUICK in this path is what was dropping the servo to
    // SAFE_OFF (then surfacing as a phantom "mastering lost" next run).
    stopMotion();
#if defined(HAVE_DRFL) && HAVE_DRFL
    // Authority release is handled by close_connection() in disconnect().
#endif
    engaged_.store(false);
    LOG_I("Robot disengaged (soft stop, authority will release on disconnect).");
}

bool DrflRobotController::moveHome(const RobotPose& safe) {
    if (!connected_.load()) return false;
    LOG_I("Robot moveHome -> (%.1f, %.1f, %.1f, %.1f, %.1f, %.1f)",
          safe.x, safe.y, safe.z, safe.rx, safe.ry, safe.rz);

#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "DRFL move(home): no access control authority";
        LOG_E("%s (grant=%d, state=%d)",
              last_error_.c_str(), g_access_grant.load(), g_robot_state.load());
        return false;
    }
    // Profile is intentionally very gentle so the joint-speed / collision
    // supervisions are unlikely to trip on the initial approach.
    //
    // Doosan posx convention: target[3..5] are (W, P, R) - Doosan ZYZ'
    // intrinsic Euler in degrees. Our RobotPose names them rx/ry/rz for
    // historical reasons but maps to W/P/R one-for-one (see RobotPose.hpp).
    float target[6] = { (float)safe.x, (float)safe.y, (float)safe.z,
                        (float)safe.rx, (float)safe.ry, (float)safe.rz };

    // IMPORTANT: do NOT add an mwait() after this move - mwait() after a
    // blended motion can trip Doosan alarm 5.7056 "Standstill status
    // violated". We also avoid setting a blending radius (radius=0)
    // and a blending type beyond the SDK default for the same reason:
    // a blended move followed by any non-motion command on 1.33.3 is
    // a known foot-gun. To wait for completion we poll the monitoring
    // state until STATE_STANDBY.
    if (cfg_.home_use_movejx) {
        // movejx: joint-space interpolation to a Cartesian target.
        // Immune to Cartesian path singularities (which trip alarm 3205
        // -> SAFE_OFF 7056 on movel for poses like Ry=96 deg).
        // Use the short overload form: only fTargetVel / fTargetAcc;
        // the SDK fills the rest with safe defaults (ABSOLUTE, BASE,
        // radius=0, default blending type).
        const unsigned char sol_space = 0;
        const float jvel   = 20.0f;  // deg/s
        const float jaccel = 60.0f;  // deg/s^2
        if (!p_->drfl.movejx(target, sol_space, jvel, jaccel)) {
            last_error_ = "DRFL movejx(home) failed (state="
                        + std::to_string(g_robot_state.load()) + ")";
            LOG_E("%s", last_error_.c_str());
            return false;
        }
    } else {
        // Cartesian linear approach. Sensitive to wrist singularities.
        // Short overload form, no blending.
        float vel[2]   = { 20.0f, 5.0f };    // mm/s, deg/s
        float accel[2] = { 50.0f, 20.0f };
        if (!p_->drfl.movel(target, vel, accel)) {
            last_error_ = "DRFL movel(home) failed (state="
                        + std::to_string(g_robot_state.load()) + ")";
            LOG_E("%s", last_error_.c_str());
            return false;
        }
    }

    // Wait for STATE_STANDBY rather than mwait(). Polling the
    // monitoring state lets us return only once the controller has
    // fully settled, without ever issuing a non-motion DRFL call while
    // the joints might still be stabilising.
    if (!waitFor([] {
            return g_robot_state.load() == static_cast<int>(STATE_STANDBY);
        }, 30.0)) {
        LOG_W("moveHome: STANDBY not reached after motion (state=%d).",
              g_robot_state.load());
    }
    // The robot is at zero velocity now; next active tick will refresh.
    last_was_zero_ = true;
#else
    // Simulator.
    std::lock_guard<std::mutex> lock(pose_mx_);
    current_pose_ = safe;
#endif
    return true;
}

// Apply a TCP definition that ALREADY exists on the Doosan controller.
// Wraps DRFL set_tcp(name). Returns false if DRFL rejects the call -
// typically because the named entry does not exist on the controller.
// Never creates or modifies TCP entries.
bool DrflRobotController::setTcp(const std::string& name) {
    if (name.empty()) {
        LOG_I("TCP application skipped: empty name");
        return true;
    }
    if (!connected_.load()) {
        last_error_ = "setTcp: not connected";
        LOG_E("%s (name='%s')", last_error_.c_str(), name.c_str());
        return false;
    }
    LOG_I("Applying TCP: %s", name.c_str());
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "setTcp: no access control authority";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
    if (!p_->drfl.set_tcp(name.c_str())) {
        last_error_ = "DRFL set_tcp() refused name '" + name +
                      "'. The TCP MUST already exist on the controller "
                      "(pendant -> Setting -> Robot -> TCP). This program "
                      "does not create TCP entries.";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
#else
    LOG_I("[SIM] set_tcp(\"%s\")", name.c_str());
#endif
    return true;
}

// Apply a Tool Weight definition that ALREADY exists on the controller.
// Wraps DRFL set_tool(name). Same constraints as setTcp.
bool DrflRobotController::setToolWeight(const std::string& name) {
    if (name.empty()) {
        LOG_I("Tool Weight application skipped: empty name");
        return true;
    }
    if (!connected_.load()) {
        last_error_ = "setToolWeight: not connected";
        LOG_E("%s (name='%s')", last_error_.c_str(), name.c_str());
        return false;
    }
    LOG_I("Applying Tool Weight: %s", name.c_str());
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "setToolWeight: no access control authority";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
    if (!p_->drfl.set_tool(name.c_str())) {
        last_error_ = "DRFL set_tool() refused name '" + name +
                      "'. The Tool Weight MUST already exist on the "
                      "controller (pendant -> Setting -> Robot -> Tool). "
                      "This program does not create Tool Weight entries.";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
#else
    LOG_I("[SIM] set_tool(\"%s\")", name.c_str());
#endif
    return true;
}

// Drive a single Tool Digital Output. `index` is 1-based to match the
// pendant labelling (O01, O02, ...). Used by ToolIoGripperController to
// pulse end-effector valves wired to the Doosan tool flange.
bool DrflRobotController::setToolDigitalOutput(int index, bool value) {
    if (!connected_.load()) {
        last_error_ = "setToolDigitalOutput: not connected";
        LOG_E("%s (O%02d=%d)", last_error_.c_str(), index, value ? 1 : 0);
        return false;
    }
    if (index < 1) {
        last_error_ = "setToolDigitalOutput: invalid 1-based index";
        LOG_E("%s (got %d)", last_error_.c_str(), index);
        return false;
    }
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "setToolDigitalOutput: no access control authority";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
    // DRFL enum: GPIO_TOOL_DIGITAL_INDEX_1 .. GPIO_TOOL_DIGITAL_INDEX_N
    // map sequentially. Convert from the 1-based pendant index.
    const GPIO_TOOL_DIGITAL_INDEX e_index =
        static_cast<GPIO_TOOL_DIGITAL_INDEX>(
            GPIO_TOOL_DIGITAL_INDEX_1 + (index - 1));
    if (!p_->drfl.set_tool_digital_output(e_index, value ? TRUE : FALSE)) {
        last_error_ = "DRFL set_tool_digital_output() failed";
        LOG_E("%s (O%02d=%d)", last_error_.c_str(), index, value ? 1 : 0);
        return false;
    }
#else
    LOG_I("[SIM] set_tool_digital_output(O%02d, %s)",
          index, value ? "ON" : "OFF");
#endif
    return true;
}

// Drive a CONTROLLER box Digital Output. Same 1-based pendant index
// convention as setToolDigitalOutput(), but it routes to the controller
// cabinet DOs (GPIO_CTRLBOX_*) via set_digital_output() instead of the
// flange DOs. Used by DigitalIoGripperController when [gripper].io_scope
// = "controller" - which is the case for the SoftHand on this cell.
bool DrflRobotController::setControllerDigitalOutput(int index, bool value) {
    if (!connected_.load()) {
        last_error_ = "setControllerDigitalOutput: not connected";
        LOG_E("%s (O%02d=%d)", last_error_.c_str(), index, value ? 1 : 0);
        return false;
    }
    if (index < 1) {
        last_error_ = "setControllerDigitalOutput: invalid 1-based index";
        LOG_E("%s (got %d)", last_error_.c_str(), index);
        return false;
    }
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (g_access_grant.load() != 1) {
        last_error_ = "setControllerDigitalOutput: no access control authority";
        LOG_E("%s", last_error_.c_str());
        return false;
    }
    // DRFL enum: GPIO_CTRLBOX_DIGITAL_INDEX_1 .. GPIO_CTRLBOX_DIGITAL_INDEX_N
    // map sequentially. Convert from the 1-based pendant index.
    const GPIO_CTRLBOX_DIGITAL_INDEX e_index =
        static_cast<GPIO_CTRLBOX_DIGITAL_INDEX>(
            GPIO_CTRLBOX_DIGITAL_INDEX_1 + (index - 1));
    if (!p_->drfl.set_digital_output(e_index, value ? TRUE : FALSE)) {
        last_error_ = "DRFL set_digital_output() failed";
        LOG_E("%s (O%02d=%d)", last_error_.c_str(), index, value ? 1 : 0);
        return false;
    }
#else
    LOG_I("[SIM] set_digital_output(O%02d, %s)  (controller box)",
          index, value ? "ON" : "OFF");
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

    // Deadband: residual EMA / sensor jitter routinely produces sub-mm/s
    // twists when the hand is held still. Treat anything below the
    // thresholds as zero and emit ONE speedl(0) on the falling edge,
    // then go quiet. Without this we spam speedl() at 60 Hz with tiny
    // non-zero values, which is one of the conditions that trip alarm
    // 5.7056 "Standstill status violated" on Doosan 1.33.3.
    constexpr double kLinDeadband_mm_s  = 0.5;
    constexpr double kAngDeadband_deg_s = 0.5;
    const bool below_lin = std::abs(twist[0]) < kLinDeadband_mm_s &&
                           std::abs(twist[1]) < kLinDeadband_mm_s &&
                           std::abs(twist[2]) < kLinDeadband_mm_s;
    const bool below_ang = std::abs(twist[3]) < kAngDeadband_deg_s &&
                           std::abs(twist[4]) < kAngDeadband_deg_s &&
                           std::abs(twist[5]) < kAngDeadband_deg_s;

#if defined(HAVE_DRFL) && HAVE_DRFL
    float accel[2] = { (float)cfg_.max_lin_accel, (float)cfg_.max_ang_accel };
    // Duration tuned slightly larger than the 60 Hz loop period so that
    // a single lost packet doesn't desync the streaming watchdog.
    constexpr float kSpeedlDuration_s = 0.2f;

    if (below_lin && below_ang) {
        if (!last_was_zero_) {
            float vel[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
            p_->drfl.speedl(vel, accel, kSpeedlDuration_s);
            last_was_zero_ = true;
        }
        return true;
    }

    last_was_zero_ = false;
    float vel[6] = { (float)twist[0], (float)twist[1], (float)twist[2],
                     (float)twist[3], (float)twist[4], (float)twist[5] };
    if (!p_->drfl.speedl(vel, accel, kSpeedlDuration_s)) {
        last_error_ = "DRFL speedl failed";
        return false;
    }
#else
    (void)below_lin; (void)below_ang;
#endif
    return true;
}

void DrflRobotController::stopMotion() {
    // SOFT pause: stream a single zero Cartesian velocity and then go
    // quiet. The caller (Application::tick) only invokes us on the
    // falling-edge active -> passive transition, NEVER in a 60 Hz loop -
    // sustained speedl(0) spam was contributing to alarm 5.7056.
    {
        std::lock_guard<std::mutex> lock(pose_mx_);
        last_twist_ = {0, 0, 0, 0, 0, 0};
    }
    if (last_was_zero_) return;          // already at rest, do nothing
    if (cfg_.dryrun_robot) {
        last_was_zero_ = true;
        return;
    }
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (connected_.load() && engaged_.load()) {
        float vel[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        float accel[2] = { (float)cfg_.max_lin_accel,
                           (float)cfg_.max_ang_accel };
        p_->drfl.speedl(vel, accel, 0.2f);
    }
#endif
    last_was_zero_ = true;
    LOG_I("Robot: soft stop (speedl zero) issued.");
}

void DrflRobotController::emergencyStop() {
    // HARD halt for true faults only. STOP_TYPE_QUICK can drop the
    // servo to SAFE_OFF on some configurations, so restrict this to
    // genuine emergencies - never call it on the 60 Hz loop and never
    // on a clean shutdown path.
    {
        std::lock_guard<std::mutex> lock(pose_mx_);
        last_twist_ = {0, 0, 0, 0, 0, 0};
    }
    last_was_zero_ = true;
    if (cfg_.dryrun_robot) return;
#if defined(HAVE_DRFL) && HAVE_DRFL
    if (connected_.load() && engaged_.load()) {
        p_->drfl.stop(STOP_TYPE_QUICK);
    }
#endif
}

bool DrflRobotController::sendCartesianMicroMove(const RobotPose& target,
                                                 double lin_vel, double ang_vel,
                                                 double lin_acc, double ang_acc,
                                                 double blending_radius_mm) {
    if (!connected_.load() || !engaged_.load()) return false;

    {
        // Keep the simulator pose in sync for UI / dryrun.
        std::lock_guard<std::mutex> lock(pose_mx_);
        current_pose_ = target;
    }
    if (cfg_.dryrun_robot) return true;

#if !defined(HAVE_DRFL) || !HAVE_DRFL
    (void)lin_vel; (void)ang_vel; (void)lin_acc; (void)ang_acc;
    (void)blending_radius_mm;
    return true;
#else
    // amovel: ASYNC, NON-BLOCKING movel. Signature (DRFL 1.33.3):
    //   amovel(target[6], vel[2], acc[2],
    //          time = 0,
    //          MOVE_MODE eMoveMode = ABSOLUTE,
    //          MOVE_REFERENCE eMoveReference = BASE,
    //          BLENDING_SPEED_TYPE eBlendingType = DUPLICATE,
    //          DR_MV_APP eAppType = DR_MV_APP_NONE)
    //
    // No blending RADIUS parameter on amovel - blending behaviour is
    // controlled solely by BLENDING_SPEED_TYPE. We map the caller's
    // requested radius onto the enum:
    //   radius > 0  -> DUPLICATE (smooth velocity across segments)
    //   radius == 0 -> OVERRIDE  (each segment is independent;
    //                             controller decelerates to zero at end)
    //
    // The caller (Application) controls the actual radius value via
    // config; the radius_mm value is logged here for visibility but not
    // passed to the SDK directly.
    //
    // We MUST NOT call mwait() here - mwait() after any movel-family
    // call is a documented trigger for alarm 5.7056.
    const BLENDING_SPEED_TYPE blending = (blending_radius_mm > 0.0)
        ? BLENDING_SPEED_TYPE_DUPLICATE
        : BLENDING_SPEED_TYPE_OVERRIDE;

    float pose[6] = { (float)target.x,  (float)target.y,  (float)target.z,
                      (float)target.rx, (float)target.ry, (float)target.rz };
    float vel[2]  = { (float)lin_vel, (float)ang_vel };
    float acc[2]  = { (float)lin_acc, (float)ang_acc };
    if (!p_->drfl.amovel(pose, vel, acc, 0.0f,
                         MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE,
                         blending, DR_MV_APP_NONE)) {
        last_error_ = "DRFL amovel(micro) failed";
        return false;
    }
    last_was_zero_ = false;
#endif
    return true;
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

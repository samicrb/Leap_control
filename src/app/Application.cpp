#include "app/Application.hpp"
#include "input/KeyboardButton.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <thread>

namespace dgd {

namespace {
double nowSeconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

// Distinguish "the robot itself is misbehaving and must be hard-halted"
// from "the operator pulled their hands away / the sensor isn't ready
// / we hit a workspace edge". Only the former justifies the controller
// STOP_TYPE_QUICK; the latter cases are just a soft pause.
bool isCriticalFault(FaultReason r) {
    switch (r) {
        case FaultReason::RobotError:
        case FaultReason::InternalError:
            return true;
        case FaultReason::None:
        case FaultReason::SensorDisconnected:
        case FaultReason::LeftHandLost:
        case FaultReason::RightHandLost:
        case FaultReason::RightHandPostureInvalid:
        case FaultReason::WorkspaceLimit:
            return false;
    }
    return false;
}

// "Active" states stream velocity commands at the loop rate. Every
// other state is passive (no velocity stream) - we only need to issue
// a soft stop on the falling edge active -> passive.
bool isActiveState(DemoState s) {
    return s == DemoState::PositionControl ||
           s == DemoState::OrientationControl;
}
} // namespace

Application::Application(const Config& cfg,
                         ILeapSource& sensor,
                         IRobotController& robot,
                         IGripperController& gripper,
                         IExternalButton& button)
    : cfg_(cfg), sensor_(sensor), robot_(robot), gripper_(gripper), button_(button),
      interpreter_(cfg),
      sm_(cfg),
      guard_(cfg, {cfg.safe_x, cfg.safe_y, cfg.safe_z, cfg.safe_rx, cfg.safe_ry, cfg.safe_rz}),
      ui_(cfg) {}

bool Application::initialise() {
    LOG_I("Application: initialising.");

    if (!sensor_.start()) {
        LOG_W("Sensor did not start cleanly - continuing in degraded mode.");
    }
    if (!button_.start()) {
        LOG_E("Button source failed to start.");
        return false;
    }
    if (!robot_.connect(cfg_.robot_ip, cfg_.robot_port, cfg_.connect_timeout_s)) {
        LOG_E("Robot connection failed: %s", robot_.lastError().c_str());
        return false;
    }
    if (!robot_.engage()) {
        LOG_E("Robot engage failed: %s", robot_.lastError().c_str());
        return false;
    }
    RobotPose safe{cfg_.safe_x, cfg_.safe_y, cfg_.safe_z,
                   cfg_.safe_rx, cfg_.safe_ry, cfg_.safe_rz};
    if (cfg_.skip_move_home) {
        LOG_I("moveHome skipped (robot.skip_move_home=true).");
    } else if (!robot_.moveHome(safe)) {
        LOG_W("moveHome failed: %s", robot_.lastError().c_str());
    }
    if (!gripper_.connect()) {
        LOG_W("Gripper connect failed: %s", gripper_.lastError().c_str());
    }
    return true;
}

void Application::stop() { running_.store(false); }

int Application::run() {
    running_.store(true);
    const auto period = std::chrono::duration<double>(loopPeriod());
    LOG_I("Application: entering main loop at %d Hz.", cfg_.loop_rate_hz);

    // Keyboard button also doubles as shutdown source (Q/ESC).
    auto* keyboard = dynamic_cast<KeyboardButton*>(&button_);

    while (running_.load()) {
        auto loop_start = std::chrono::steady_clock::now();
        tick(nowSeconds());

        if (keyboard && keyboard->shutdownRequested()) {
            LOG_I("Shutdown requested via keyboard.");
            running_.store(false);
            break;
        }

        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        auto remaining = period - elapsed;
        if (remaining.count() > 0.0) {
            std::this_thread::sleep_for(remaining);
        }
    }

    LOG_I("Application: exiting main loop.");
    // Normal shutdown path: just disconnect. The adapter's disconnect()
    // chains to disengage() -> stopMotion() (soft speedl-zero) and then
    // close_connection() which frees DRFL access authority cleanly. We
    // deliberately do NOT call emergencyStop() here - normal shutdown is
    // not a critical fault and STOP_TYPE_QUICK can drop the servo to
    // SAFE_OFF, which then masks itself as a mastering loss next run.
    robot_.disconnect();
    gripper_.disconnect();
    sensor_.stop();
    button_.stop();
    return 0;
}

void Application::tick(double now_s) {
    // 1. Pull the latest frame. If nothing new, use the prior one but
    //    mark it as stale (sensor_connected stays, but freshness field
    //    in the report still reflects reality).
    HandFrame frame{};
    bool fresh = sensor_.pollLatest(frame);
    if (!fresh) {
        frame = last_frame_;
        if (!sensor_.isConnected()) frame.sensor_connected = false;
    }
    last_frame_ = frame;

    // 2. Gesture interpretation.
    GestureReport report = interpreter_.update(frame, now_s);
    report.freshFrame = fresh;

    // 3. State machine.
    bool btn = button_.isActive();
    CommandOutput cmd = sm_.step(report, btn, now_s);

    if (cmd.captureReference) interpreter_.captureReference();

    // 4. Execute commands.
    //    Transition-driven design: we only send robot commands on the
    //    state edges (active <-> passive) and during active streaming.
    //    Sitting in a stable passive state (IDLE / READY / RECENTER /
    //    GRIPPER / non-critical FAULT) sends NOTHING - which avoids the
    //    repeated speedl(0) / stopMotion() at 60 Hz that was tripping
    //    Doosan alarm 5.7056 "Standstill status is violated".
    const DemoState  cur_state    = sm_.state();
    const FaultReason fault_now   = sm_.faultReason();
    const bool        cur_active  = isActiveState(cur_state);
    const bool        prev_active = isActiveState(prev_state_);
    const bool        critical    = (cur_state == DemoState::Fault) &&
                                    isCriticalFault(fault_now);

    bool touched_limit = false;

    if (critical && !fault_emergency_issued_) {
        // One-shot hard halt on entering a critical fault.
        LOG_W("CRITICAL fault (%s) - emergencyStop issued.",
              faultReasonText(fault_now));
        robot_.emergencyStop();
        fault_emergency_issued_ = true;
    } else if (cur_active) {
        // Streaming control. The adapter applies a velocity deadband and
        // skips speedl() when the commanded twist is effectively zero, so
        // this call is safe at the loop rate even when the operator's
        // hand is momentarily still.
        if (!prev_active) {
            LOG_I("State %s -> %s : entering active control.",
                  stateName(prev_state_), stateName(cur_state));
        }
        std::array<double, 6> twist = {
            cmd.linear_velocity[0],  cmd.linear_velocity[1],  cmd.linear_velocity[2],
            cmd.angular_velocity[0], cmd.angular_velocity[1], cmd.angular_velocity[2]
        };
        RobotPose pose;
        if (robot_.getCurrentPose(pose)) {
            touched_limit = guard_.clamp(pose, twist, loopPeriod());
        }
        robot_.sendCartesianVelocity(twist);
        fault_emergency_issued_ = false;
    } else if (prev_active) {
        // Falling edge active -> passive. Single soft stop to decelerate
        // cleanly; subsequent passive ticks send nothing.
        LOG_I("State %s -> %s : soft stop sent.",
              stateName(prev_state_), stateName(cur_state));
        robot_.stopMotion();
        fault_emergency_issued_ = false;
    } else {
        // Passive -> passive (stable). Do NOT touch the robot.
        // (Critical-fault re-ticks also fall here once
        // fault_emergency_issued_ has latched.)
        fault_emergency_issued_ = critical ? fault_emergency_issued_ : false;
    }

    prev_state_ = cur_state;

    if (cmd.openGripper)  gripper_.open();
    if (cmd.closeGripper) gripper_.close();

    // 5. UI refresh.
    RobotPose pose_now;
    robot_.getCurrentPose(pose_now);

    ConsoleUI::Frame uiframe;
    uiframe.state         = sm_.state();
    uiframe.fault         = sm_.faultReason();
    uiframe.button_active = btn;
    uiframe.sensor_ok     = report.sensor_ok;
    uiframe.mode_text     = cmd.ui_mode_text;
    uiframe.status_text   = cmd.ui_status_text;
    uiframe.prompt_text   = cmd.ui_prompt_text;
    uiframe.gesture       = report;
    uiframe.pose          = pose_now;
    uiframe.gripper       = gripper_.lastCommandedState();
    uiframe.workspace_limit = touched_limit;
    ui_.render(uiframe);
}

} // namespace dgd

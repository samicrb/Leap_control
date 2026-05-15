#include "app/Application.hpp"
#include "input/KeyboardButton.hpp"
#include "util/Logger.hpp"

#include <cmath>
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

// Pursuit step: pick a value along `error` that closes the gap without
// overshoot and without firing micro-steps below the arrival band.
double computeStep(double error, double min_step, double max_step,
                   double arrival_band) {
    const double abs_e = std::abs(error);
    if (abs_e < arrival_band) return 0.0;
    double step_abs = std::min(abs_e, max_step);
    if (step_abs > arrival_band) {
        step_abs = std::max(step_abs, std::min(min_step, abs_e));
    }
    return std::copysign(step_abs, error);
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
        // One-shot hard halt on entering a critical fault. Drops any
        // pending micro-motion plan; an in-flight amovel will be
        // overridden by the controller-level stop().
        LOG_W("CRITICAL fault (%s) - emergencyStop issued.",
              faultReasonText(fault_now));
        robot_.emergencyStop();
        fault_emergency_issued_         = true;
        virtual_target_initialised_     = false;
        active_entry_robot_pose_valid_  = false;
        last_commanded_target_valid_    = false;
    } else if (cur_active) {
        // Active control via the MICRO-MOTION SUPERVISOR.
        //
        // The active path issues discrete amovel commands only. Two
        // controller modes exist:
        //
        //   - PURSUIT (default, cfg_.micro_pursuit_enabled = true):
        //       The hand defines an ABSOLUTE desired target relative
        //       to active_entry_robot_pose_ (snapshotted on entry).
        //       Each scheduler tick a bounded pursuit step is applied
        //       to last_commanded_target_pose_ so it tracks
        //       desired_target_pose_ continuously. Blending across
        //       consecutive amovel segments preserves velocity at the
        //       boundaries -> visually fluid motion.
        //
        //   - LEGACY incremental (cfg_.micro_pursuit_enabled = false):
        //       Per-tick velocity*dt is integrated into a virtual
        //       target; amovel emits each new target. Kept for
        //       backward compatibility.
        //
        // In BOTH modes:
        //   - The ONLY DRFL command issued in this branch is amovel().
        //   - getCurrentPose() runs at most ONCE per active entry to
        //     seed the controller; it never interleaves with an
        //     in-flight amovel.
        //   - No stopMotion, no mode change, no mwait.
        if (!prev_active) {
            // Seed both controllers from the real TCP pose.
            RobotPose seed;
            if (robot_.getCurrentPose(seed)) {
                virtual_target_pose_           = seed;
                virtual_target_initialised_    = true;
                active_entry_robot_pose_       = seed;
                desired_target_pose_           = seed;
                last_commanded_target_pose_    = seed;
                active_entry_robot_pose_valid_ = true;
                last_commanded_target_valid_   = true;
                last_command_sent_s_           = now_s - cfg_.micro_min_period_s;
                if (cfg_.micro_pursuit_enabled) {
                    LOG_I("State %s -> %s : entering MICRO-PURSUIT "
                          "(ratio=%.2f, step=[%.1f..%.1f mm / %.1f..%.1f deg], "
                          "rate=%.1f Hz, blend=%s @ %.1f mm).",
                          stateName(prev_state_), stateName(cur_state),
                          cfg_.micro_hand_to_robot_ratio,
                          cfg_.micro_min_step_xyz_mm, cfg_.micro_max_step_xyz_mm,
                          cfg_.micro_min_step_rot_deg, cfg_.micro_max_step_rot_deg,
                          cfg_.micro_command_rate_hz,
                          cfg_.micro_blending_enabled ? "ON" : "OFF",
                          cfg_.micro_blending_radius_mm);
                } else {
                    LOG_I("State %s -> %s : entering MICRO-MOTION (legacy "
                          "incremental, rate=%.1f Hz).",
                          stateName(prev_state_), stateName(cur_state),
                          cfg_.micro_command_rate_hz);
                }
            } else {
                virtual_target_initialised_    = false;
                active_entry_robot_pose_valid_ = false;
                last_commanded_target_valid_   = false;
                LOG_W("State %s -> %s : entering active mode but could not "
                      "seed pose (getCurrentPose failed).",
                      stateName(prev_state_), stateName(cur_state));
            }
        }

        const bool ready = cfg_.micro_pursuit_enabled
            ? (active_entry_robot_pose_valid_ && last_commanded_target_valid_)
            : virtual_target_initialised_;

        if (ready) {
            // Scheduler: throttle to micro_command_rate_hz / micro_min_period_s.
            const double elapsed = now_s - last_command_sent_s_;
            const double period_floor =
                std::max(cfg_.micro_min_period_s,
                         1.0 / std::max(cfg_.micro_command_rate_hz, 0.1));

            // PURSUIT mode: recompute desired_target_pose_ every tick.
            // (Independent of scheduler - tracks the operator continuously.)
            if (cfg_.micro_pursuit_enabled) {
                // Recover the raw hand displacement from cmd.linear_velocity.
                // The state machine emits  cmd.linear_velocity = position_scale
                // * hand_displacement, so dividing back recovers the geometric
                // hand delta the operator is showing.
                const double inv_pos = (cfg_.position_scale    > 1e-6)
                                     ? 1.0 / cfg_.position_scale    : 0.0;
                const double inv_rot = (cfg_.orientation_scale > 1e-6)
                                     ? 1.0 / cfg_.orientation_scale : 0.0;
                const double r       = cfg_.micro_hand_to_robot_ratio;

                desired_target_pose_.x  = active_entry_robot_pose_.x
                    + r * cmd.linear_velocity[0]  * inv_pos;
                desired_target_pose_.y  = active_entry_robot_pose_.y
                    + r * cmd.linear_velocity[1]  * inv_pos;
                desired_target_pose_.z  = active_entry_robot_pose_.z
                    + r * cmd.linear_velocity[2]  * inv_pos;
                desired_target_pose_.rx = active_entry_robot_pose_.rx
                    + r * cmd.angular_velocity[0] * inv_rot;
                desired_target_pose_.ry = active_entry_robot_pose_.ry
                    + r * cmd.angular_velocity[1] * inv_rot;
                desired_target_pose_.rz = active_entry_robot_pose_.rz
                    + r * cmd.angular_velocity[2] * inv_rot;
            }

            if (elapsed < period_floor) {
                // Too soon since last amovel - skip. No log: 60 Hz spam.
            } else if (cfg_.micro_pursuit_enabled) {
                // --- PURSUIT step ---------------------------------------
                const double sx  = computeStep(
                    desired_target_pose_.x  - last_commanded_target_pose_.x,
                    cfg_.micro_min_step_xyz_mm, cfg_.micro_max_step_xyz_mm,
                    cfg_.micro_arrival_band_xyz_mm);
                const double sy  = computeStep(
                    desired_target_pose_.y  - last_commanded_target_pose_.y,
                    cfg_.micro_min_step_xyz_mm, cfg_.micro_max_step_xyz_mm,
                    cfg_.micro_arrival_band_xyz_mm);
                const double sz  = computeStep(
                    desired_target_pose_.z  - last_commanded_target_pose_.z,
                    cfg_.micro_min_step_xyz_mm, cfg_.micro_max_step_xyz_mm,
                    cfg_.micro_arrival_band_xyz_mm);
                const double srx = computeStep(
                    desired_target_pose_.rx - last_commanded_target_pose_.rx,
                    cfg_.micro_min_step_rot_deg, cfg_.micro_max_step_rot_deg,
                    cfg_.micro_arrival_band_rot_deg);
                const double sry = computeStep(
                    desired_target_pose_.ry - last_commanded_target_pose_.ry,
                    cfg_.micro_min_step_rot_deg, cfg_.micro_max_step_rot_deg,
                    cfg_.micro_arrival_band_rot_deg);
                const double srz = computeStep(
                    desired_target_pose_.rz - last_commanded_target_pose_.rz,
                    cfg_.micro_min_step_rot_deg, cfg_.micro_max_step_rot_deg,
                    cfg_.micro_arrival_band_rot_deg);

                const bool any_step =
                    sx != 0.0 || sy != 0.0 || sz != 0.0 ||
                    srx != 0.0 || sry != 0.0 || srz != 0.0;

                if (any_step) {
                    RobotPose pursuit_target = last_commanded_target_pose_;
                    pursuit_target.x  += sx;
                    pursuit_target.y  += sy;
                    pursuit_target.z  += sz;
                    pursuit_target.rx += srx;
                    pursuit_target.ry += sry;
                    pursuit_target.rz += srz;

                    // Adaptive blending: shrink the requested radius on
                    // a small segment so the controller doesn't try to
                    // round more than the segment itself.
                    const double seg_norm =
                        std::sqrt(sx*sx + sy*sy + sz*sz);
                    const double radius = cfg_.micro_blending_enabled
                        ? std::min(cfg_.micro_blending_radius_mm,
                                   0.4 * seg_norm)
                        : 0.0;

                    if (robot_.sendCartesianMicroMove(
                            pursuit_target,
                            cfg_.micro_lin_vel, cfg_.micro_ang_vel,
                            cfg_.micro_lin_acc, cfg_.micro_ang_acc,
                            radius)) {
                        last_commanded_target_pose_ = pursuit_target;
                        last_command_sent_s_        = now_s;
                    } else {
                        LOG_W("Pursuit micro-move refused by adapter: %s",
                              robot_.lastError().c_str());
                    }
                }
                // else: within arrival band on every axis -> nothing to do.
            } else {
                // --- LEGACY incremental mode ---------------------------
                std::array<double, 6> twist = {
                    cmd.linear_velocity[0],  cmd.linear_velocity[1],  cmd.linear_velocity[2],
                    cmd.angular_velocity[0], cmd.angular_velocity[1], cmd.angular_velocity[2]
                };
                guard_.clampSpeed(twist);

                auto bound = [](double v, double m) {
                    if (v >  m) return  m;
                    if (v < -m) return -m;
                    return v;
                };
                double dx  = bound(twist[0] * elapsed, cfg_.micro_max_delta_xyz_mm);
                double dy  = bound(twist[1] * elapsed, cfg_.micro_max_delta_xyz_mm);
                double dz  = bound(twist[2] * elapsed, cfg_.micro_max_delta_xyz_mm);
                double drx = bound(twist[3] * elapsed, cfg_.micro_max_delta_rot_deg);
                double dry = bound(twist[4] * elapsed, cfg_.micro_max_delta_rot_deg);
                double drz = bound(twist[5] * elapsed, cfg_.micro_max_delta_rot_deg);

                const bool meaningful =
                    std::abs(dx)  > cfg_.micro_deadband_mm  ||
                    std::abs(dy)  > cfg_.micro_deadband_mm  ||
                    std::abs(dz)  > cfg_.micro_deadband_mm  ||
                    std::abs(drx) > cfg_.micro_deadband_deg ||
                    std::abs(dry) > cfg_.micro_deadband_deg ||
                    std::abs(drz) > cfg_.micro_deadband_deg;

                if (meaningful) {
                    virtual_target_pose_.x  += dx;
                    virtual_target_pose_.y  += dy;
                    virtual_target_pose_.z  += dz;
                    virtual_target_pose_.rx += drx;
                    virtual_target_pose_.ry += dry;
                    virtual_target_pose_.rz += drz;
                    const double radius = cfg_.micro_blending_enabled
                                        ? cfg_.micro_blending_radius_mm : 0.0;
                    if (robot_.sendCartesianMicroMove(
                            virtual_target_pose_,
                            cfg_.micro_lin_vel, cfg_.micro_ang_vel,
                            cfg_.micro_lin_acc, cfg_.micro_ang_acc,
                            radius)) {
                        last_command_sent_s_ = now_s;
                    } else {
                        LOG_W("Micro-move refused by adapter: %s",
                              robot_.lastError().c_str());
                    }
                }
            }
        }
        fault_emergency_issued_ = false;
    } else if (prev_active) {
        // Falling edge active -> passive. Per the supervisor design we
        // simply STOP issuing new amovels. The last in-flight short
        // motion completes naturally and the controller halts smoothly
        // at its end pose. NO stopMotion(), NO speedl(0), NO
        // emergencyStop, NO mwait.
        LOG_I("State %s -> %s : ceasing micro-motion stream "
              "(last in-flight amovel will run to completion).",
              stateName(prev_state_), stateName(cur_state));
        virtual_target_initialised_    = false;
        active_entry_robot_pose_valid_ = false;
        last_commanded_target_valid_   = false;
        fault_emergency_issued_        = false;
    } else {
        // Passive -> passive (stable). Do NOT touch the robot.
        fault_emergency_issued_ = critical ? fault_emergency_issued_ : false;
    }

    prev_state_ = cur_state;

    if (cmd.openGripper)  gripper_.open();
    if (cmd.closeGripper) gripper_.close();

    // 5. UI refresh.
    //    Pose lookup is gated: NEVER while the active micro-motion
    //    pipeline is running (PositionControl / OrientationControl,
    //    where amovel commands may be in flight). Outside streaming,
    //    refresh at most every 0.5 s to keep the read load down.
    constexpr double kUiPosePollPeriod_s = 0.5;
    if (!cur_active && (now_s - last_pose_poll_s_) > kUiPosePollPeriod_s) {
        RobotPose fresh_pose;
        if (robot_.getCurrentPose(fresh_pose)) {
            last_pose_for_ui_ = fresh_pose;
            last_pose_poll_s_ = now_s;
        }
    }

    ConsoleUI::Frame uiframe;
    uiframe.state         = sm_.state();
    uiframe.fault         = sm_.faultReason();
    uiframe.button_active = btn;
    uiframe.sensor_ok     = report.sensor_ok;
    uiframe.mode_text     = cmd.ui_mode_text;
    uiframe.status_text   = cmd.ui_status_text;
    uiframe.prompt_text   = cmd.ui_prompt_text;
    uiframe.gesture       = report;
    uiframe.pose          = last_pose_for_ui_;   // cached - never refetched while active
    uiframe.gripper       = gripper_.lastCommandedState();
    uiframe.workspace_limit = touched_limit;
    ui_.render(uiframe);
}

} // namespace dgd

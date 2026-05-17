#include "app/Application.hpp"
#include "input/KeyboardButton.hpp"
#include "util/Logger.hpp"
#include "util/MathUtils.hpp"

#include <cmath>
#include <chrono>
#include <cstdio>
#include <optional>
#include <thread>
#include <utility>

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

// Clamp v to ±limit.
double clampAbs(double v, double limit) {
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

// Limit |delta| between two scalars so |v_new - v_prev| <= cap.
double limitDelta(double v_new, double v_prev, double cap) {
    const double d = v_new - v_prev;
    if (d >  cap) return v_prev + cap;
    if (d < -cap) return v_prev - cap;
    return v_new;
}

} // namespace

Application::Application(Config& cfg,
                         ILeapSource& sensor,
                         IRobotController& robot,
                         IGripperController& gripper,
                         IExternalButton& button,
                         std::string config_path)
    : cfg_(cfg), sensor_(sensor), robot_(robot), gripper_(gripper), button_(button),
      interpreter_(cfg),
      sm_(cfg),
      guard_(cfg, {cfg.safe_x, cfg.safe_y, cfg.safe_z, cfg.safe_rx, cfg.safe_ry, cfg.safe_rz}),
      ui_(cfg),
      config_path_(std::move(config_path)) {}

bool Application::initialise() {
    LOG_I("Application: initialising.");

    rebindLoggingIfNeeded();
    if (cfg_.runtime_tuning_enabled) {
        reloader_.attach(config_path_, cfg_,
            [this](const std::string& msg) {
                if (cfg_.runtime_tuning_log_changes_to_event_file) {
                    event_logger_.event(msg, false);
                }
                if (cfg_.runtime_tuning_print_changes_to_console) {
                    std::fprintf(stderr, "[TUNE] %s\n", msg.c_str());
                }
            });
        event_logger_.event("Runtime tuning enabled (watching " + config_path_ + ")");
    }
    event_logger_.event("Program started",
                        cfg_.runtime_tuning_print_changes_to_console);

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
    event_logger_.event("Program shutdown",
                        cfg_.runtime_tuning_print_changes_to_console);
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
    motion_logger_.close();
    event_logger_.close();
    reloader_.detach();
    return 0;
}

void Application::rebindLoggingIfNeeded() {
    // Open / close the loggers to match the current cfg state. Called
    // once at startup and again whenever a hot-reload flips any of the
    // top-level enable flags. NEVER fails the control loop on its own.
    const bool need_motion = cfg_.logging_enabled && cfg_.logging_motion_csv_enabled;
    const bool need_event  = cfg_.logging_enabled && cfg_.logging_event_log_enabled;

    const bool experiment_changed =
        (prior_log_directory_  != cfg_.logging_directory) ||
        (prior_experiment_name_ != cfg_.logging_experiment_name);

    if (need_motion) {
        const bool reopen = !motion_logger_.isOpen() || experiment_changed ||
                            !motion_csv_was_enabled_;
        if (reopen) motion_logger_.open(cfg_);
    } else if (motion_logger_.isOpen()) {
        motion_logger_.close();
    }

    if (need_event) {
        const bool reopen = !event_logger_.isOpen() || experiment_changed ||
                            !event_log_was_enabled_;
        if (reopen) event_logger_.open(cfg_);
    } else if (event_logger_.isOpen()) {
        event_logger_.close();
    }

    logging_was_enabled_    = cfg_.logging_enabled;
    motion_csv_was_enabled_ = cfg_.logging_motion_csv_enabled;
    event_log_was_enabled_  = cfg_.logging_event_log_enabled;
    prior_log_directory_    = cfg_.logging_directory;
    prior_experiment_name_  = cfg_.logging_experiment_name;
}

void Application::emitConsoleSummary(double now_s, bool cur_active) {
    if (!cfg_.debug_print_motion_summary) return;
    if (summary_window_start_s_ <= 0.0) summary_window_start_s_ = now_s;
    const double period = std::max(cfg_.debug_summary_period_s, 0.1);
    if (now_s - last_summary_s_ < period) return;

    const double elapsed = std::max(now_s - summary_window_start_s_, 1e-3);
    const double sent_hz    = sent_in_window_    / elapsed;
    const double skip_hz    = skipped_in_window_ / elapsed;
    const double eff_scale  = cfg_.position_scale *
                              (cfg_.micro_pursuit_enabled
                                  ? cfg_.micro_hand_to_robot_ratio : 1.0);
    std::fprintf(stderr,
        "[RUN] active=%d tracking=%d sent=%.1f/s skipped=%.1f/s "
        "scale=%.2f vel=%.0f acc=%.0f step=%.1f blend=%.1f smooth=%.2f\n",
        cur_active ? 1 : 0,
        last_frame_.sensor_connected ? 1 : 0,
        sent_hz, skip_hz, eff_scale,
        cfg_.micro_lin_vel, cfg_.micro_lin_acc,
        cfg_.micro_max_step_xyz_mm,
        cfg_.micro_blending_enabled ? cfg_.micro_blending_radius_mm : 0.0,
        cfg_.smoothing_alpha);

    last_summary_s_         = now_s;
    summary_window_start_s_ = now_s;
    sent_in_window_         = 0;
    skipped_in_window_      = 0;
}

void Application::tick(double now_s) {
    // 0. Hot-reload pass. Reads .ini mtime at most once per poll
    //    interval; reparses only on a real change. Mutations are
    //    committed in this single call so no other code observes a
    //    half-updated Config struct.
    if (cfg_.runtime_tuning_enabled) {
        if (reloader_.poll(now_s)) {
            rebindLoggingIfNeeded();
        }
    }

    const double dt_ms = (last_tick_s_ > 0.0)
                       ? (now_s - last_tick_s_) * 1000.0 : 0.0;
    last_tick_s_ = now_s;

    const bool prev_sensor = last_frame_.sensor_connected;

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

    if (prev_sensor != frame.sensor_connected) {
        event_logger_.event(frame.sensor_connected
                                ? "Tracking recovered"
                                : "Tracking lost: freezing motion",
                            cfg_.runtime_tuning_print_changes_to_console);
    }

    // 2. Gesture interpretation.
    GestureReport report = interpreter_.update(frame, now_s);
    report.freshFrame = fresh;

    // --- Tracking-stability gate (graded) --------------------------------
    // last_valid_tracking_s_ ratchets forward on every tick that delivered
    // a fresh Leap frame AND the right hand was actually present. We then
    // classify the CURRENT tick by the age of that stamp:
    //   tracking_recent    : age <= frame_timeout_s   (treat as normal)
    //   brief_loss_active  : age in [frame_timeout, brief_loss_timeout]
    //                        and tolerance is enabled
    //   hard_tracking_loss : age > brief_loss_timeout or tolerance off
    // This is a single-source-of-truth for what the rest of the tick
    // does about tracking. It replaces the previous boolean
    // tracking_valid_now which faulted out on a single dropped frame.
    if (report.freshFrame && report.sensor_ok && report.rightPresent) {
        last_valid_tracking_s_ = now_s;
    }
    const double tracking_loss_age_s =
        (last_valid_tracking_s_ < 0.0) ? 1e9
                                       : (now_s - last_valid_tracking_s_);
    const bool tolerance_on  = cfg_.tracking_loss_tolerance_enabled;
    const bool tracking_recent =
        report.sensor_ok && report.rightPresent &&
        tracking_loss_age_s <= cfg_.tracking_loss_frame_timeout_s &&
        last_valid_tracking_s_ >= 0.0;
    const bool brief_loss_active =
        tolerance_on && !tracking_recent &&
        last_valid_tracking_s_ >= 0.0 &&
        tracking_loss_age_s <= cfg_.tracking_loss_brief_timeout_s;
    const bool hard_tracking_loss =
        !tracking_recent && !brief_loss_active;

    // --- Brief-loss hold state machine -----------------------------------
    bool reanchor_performed = false;

    // Hold entry: first tick that brief_loss_active is true.
    if (brief_loss_active && !tracking_hold_active_) {
        tracking_hold_active_   = true;
        tracking_hold_entered_s_ = now_s;
        tracking_recent_again_s_ = -1.0;
        event_logger_.event("tracking lost: entering temporary hold",
                            cfg_.runtime_tuning_print_changes_to_console);
        if (cfg_.tracking_loss_freeze_target_on_loss) {
            event_logger_.event("brief tracking loss: freezing target");
        }
        // Optional controlled stop: drive velocity to zero via the
        // existing ramp_to_zero machinery. NO speedl, NO stopMotion -
        // ramp tail keeps amovel-only contract.
        if (cfg_.tracking_loss_ramp_to_zero_on_loss &&
            cfg_.micro_velocity_filter_enabled &&
            active_entry_robot_pose_valid_ && !ramp_to_zero_) {
            ramp_to_zero_ = true;
            ramp_start_s_ = now_s;
        }
    }

    // Hard escalation: brief-loss window exceeded.
    if (hard_tracking_loss && tracking_hold_active_) {
        tracking_hold_active_   = false;
        tracking_recent_again_s_ = -1.0;
        event_logger_.event(
            "tracking loss exceeded timeout: escalating to strict behavior",
            cfg_.runtime_tuning_print_changes_to_console);
    }

    // Recovery: tracking is back AND it has held long enough.
    if (tracking_hold_active_ && tracking_recent) {
        if (tracking_recent_again_s_ < 0.0) {
            tracking_recent_again_s_ = now_s;
            event_logger_.event(
                "tracking recovered: waiting stability window");
        }
        const double stable_age = now_s - tracking_recent_again_s_;
        if (stable_age >= cfg_.tracking_loss_recovery_stability_s) {
            // Re-anchor: take CURRENT recovered hand pose as the new
            // gesture reference and pin the robot active-entry pose to
            // last_commanded_target_pose_ so desired_target = active_entry
            // + ratio * 0 = active_entry. This single operation prevents
            // the post-recovery jump caused by accumulated hand motion
            // during the dropout.
            if (cfg_.tracking_loss_reanchor_on_recovery &&
                active_entry_robot_pose_valid_) {
                interpreter_.captureReference();
                active_entry_robot_pose_    = last_commanded_target_pose_;
                desired_target_pose_        = last_commanded_target_pose_;
                event_logger_.event("tracking recovered: re-anchoring hand reference",
                                    cfg_.runtime_tuning_print_changes_to_console);
                reanchor_performed = true;
            }
            if (cfg_.tracking_loss_reset_velocity_on_recovery) {
                filtered_velocity_.fill(0.0);
                last_accel_.fill(0.0);
            }
            ramp_to_zero_ = false;
            tracking_hold_active_    = false;
            tracking_recent_again_s_ = -1.0;
            recovery_soft_commands_remaining_ =
                std::max(0, cfg_.tracking_loss_recovery_soft_commands);
            event_logger_.event("tracking recovered: resuming position control",
                                cfg_.runtime_tuning_print_changes_to_console);
        }
    } else if (tracking_hold_active_ && !tracking_recent) {
        // Wobble during recovery - require the stability window to start over.
        tracking_recent_again_s_ = -1.0;
    }

    // Keep the legacy tracking_valid_since_s_ logic so the pursuit re-arm
    // ("first entry into active") still works. This time window is about
    // INITIAL arm, separate from brief-loss recovery.
    const bool tracking_valid_now = tracking_recent;
    if (tracking_valid_now != tracking_valid_last_) {
        if (tracking_valid_now) {
            tracking_valid_since_s_ = now_s;
        } else {
            tracking_valid_since_s_ = -1.0;
            pursuit_armed_          = false;
        }
        tracking_valid_last_ = tracking_valid_now;
    }
    const double tracking_age_s =
        (tracking_valid_now && tracking_valid_since_s_ >= 0.0)
            ? (now_s - tracking_valid_since_s_) : 0.0;
    const bool tracking_stable =
        tracking_valid_now &&
        tracking_age_s >= cfg_.micro_tracking_recovery_time_s;
    if (tracking_stable && !pursuit_armed_) {
        pursuit_armed_ = true;
        event_logger_.event("tracking stable: re-arm position control",
                            cfg_.runtime_tuning_print_changes_to_console);
    }

    // 3. State machine.
    static bool prev_btn = false;
    bool btn = button_.isActive();
    if (btn != prev_btn) {
        event_logger_.event(btn ? "Deadman pressed" : "Deadman released",
                            cfg_.runtime_tuning_print_changes_to_console);
        prev_btn = btn;
    }
    // While tolerance is armed and we're inside the brief-loss window,
    // suppress SM hand-loss FAULTs. The SM remains in POSITION /
    // ORIENTATION, the Application freezes the pursuit, and the operator
    // does NOT have to lift-and-show hands on recovery. Sensor disconnects
    // are still raised (controller safety contract unchanged).
    sm_.setSuppressHandLossFault(brief_loss_active || tracking_hold_active_);
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

    // Diagnostic accumulators - filled by the branches below so the
    // motion CSV row carries exactly what the controller saw this tick.
    bool        sample_command_sent = false;
    const char* sample_skip_reason  = "";
    std::optional<std::array<double, 6>> sample_desired_target;
    std::optional<std::array<double, 6>> sample_last_commanded;
    std::optional<std::array<double, 6>> sample_commanded_target;
    double      sample_step_xyz = 0.0;
    double      sample_step_rot = 0.0;
    // Extended diagnostics for this tick.
    double sample_command_interval_ms = 0.0;
    double sample_scheduler_elapsed_ms = 0.0;
    double sample_prev_motion_est_ms  = 0.0;
    bool   sample_backlog_guard_active = false;
    std::optional<std::array<double, 6>> sample_raw_error;
    std::optional<std::array<double, 6>> sample_desired_velocity;
    std::optional<std::array<double, 6>> sample_filtered_velocity;
    std::optional<std::array<double, 6>> sample_limited_accel;
    double sample_raw_step_xyz_mm = 0.0;
    double sample_raw_step_rot_deg = 0.0;
    bool   sample_velocity_deadband_applied = false;
    bool   sample_jerk_limit_applied        = false;
    bool   sample_accel_limit_applied       = false;
    bool   sample_step_norm_clipped         = false;
    bool   sample_recovery_step_limited_local = false;

    // Hard tracking gate: if tracking is not stable AND we're not in
    // a controlled ramp-to-zero tail, freeze the active branch entirely.
    // No state update, no command emission. The previous segment (if
    // any) is allowed to finish executing on the controller.
    const bool active_branch_eligible =
        critical || (cur_active && tracking_stable) || ramp_to_zero_;

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
        ramp_to_zero_                   = false;
        filtered_velocity_.fill(0.0);
        last_accel_.fill(0.0);
    } else if (cur_active && tracking_hold_active_ && !ramp_to_zero_) {
        // Brief tracking-loss tolerance: SM still says cur_active, but
        // we're masking the hand-loss FAULT and waiting for recovery.
        // Freeze: do not update desired_target, do not seed, do not emit.
        // If ramp_to_zero_on_loss is true the ramp_to_zero_ flag was set
        // and the velocity-filter branch will run with a zero desired
        // target -> filtered velocity decays naturally.
        sample_skip_reason = "tracking_hold_freeze";
        fault_emergency_issued_ = false;
    } else if (cur_active && !tracking_recent && !ramp_to_zero_) {
        // Tracking is hard-invalid this tick (either tolerance is off,
        // or we just escalated). No seeding, no emit, no integration.
        sample_skip_reason = tolerance_on ? "hard_tracking_loss"
                                          : "tracking_invalid";
        fault_emergency_issued_ = false;
    } else if (cur_active && !tracking_stable && !ramp_to_zero_) {
        // Cold-start: we have valid recent tracking but it has not been
        // continuously valid long enough to safely re-arm pursuit.
        sample_skip_reason = "tracking_recovery_wait";
        fault_emergency_issued_ = false;
    } else if (active_branch_eligible && (cur_active || ramp_to_zero_)) {
        // Active control via the MICRO-MOTION SUPERVISOR (with optional
        // velocity-smoothing tail when ramp_to_zero_ is latched).
        // ----------------------------------------------------------
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
        // Seeding gate: only fire on the rising edge (prev_active=false
        // -> cur_active=true), AND only when tracking has been stable
        // for cfg_.micro_tracking_recovery_time_s. Otherwise we'd seed
        // from a pose taken right after a wobble and immediately drive
        // the pursuit toward a phantom desired target.
        if (!prev_active && cur_active && tracking_stable) {
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
                // Reset velocity smoothing state - the robot is at rest
                // at active entry.
                filtered_velocity_.fill(0.0);
                last_accel_.fill(0.0);
                last_vel_update_s_ = now_s;
                ramp_to_zero_      = false;
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

        if (!ready) {
            sample_skip_reason = "not_ready";
        }

        if (ready) {
            // Scheduler: throttle to micro_command_rate_hz / micro_min_period_s.
            const double elapsed = now_s - last_command_sent_s_;
            const double period_floor =
                std::max(cfg_.micro_min_period_s,
                         1.0 / std::max(cfg_.micro_command_rate_hz, 0.1));

            // PURSUIT mode: recompute desired_target_pose_ every tick.
            // (Independent of scheduler - tracks the operator continuously.)
            if (cfg_.micro_pursuit_enabled && cur_active) {
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
            } else if (ramp_to_zero_) {
                // Ramp-to-zero: hold the desired target at the current
                // commanded position. Zero error -> zero desired
                // velocity -> filtered velocity decays under accel/jerk
                // caps. The amovel tail naturally decelerates.
                desired_target_pose_ = last_commanded_target_pose_;
            }

            if (last_commanded_target_valid_) {
                sample_last_commanded = last_commanded_target_pose_.toArray();
            }
            if (cfg_.micro_pursuit_enabled || ramp_to_zero_) {
                sample_desired_target = desired_target_pose_.toArray();
            }

            sample_scheduler_elapsed_ms = elapsed * 1000.0;

            // Backlog-guard estimate of the previous segment execution.
            const double t_est_lin = cfg_.micro_lin_vel > 1e-3
                ? last_emitted_step_xyz_mm_  / cfg_.micro_lin_vel : 0.0;
            const double t_est_ang = cfg_.micro_ang_vel > 1e-3
                ? last_emitted_step_rot_deg_ / cfg_.micro_ang_vel : 0.0;
            const double t_est_s = std::max(t_est_lin, t_est_ang);
            sample_prev_motion_est_ms = t_est_s * 1000.0;
            const bool backlog_guard_engaged =
                cfg_.prevent_command_backlog &&
                last_emitted_step_xyz_mm_ + last_emitted_step_rot_deg_ > 1e-6 &&
                elapsed < cfg_.min_motion_completion_ratio * t_est_s &&
                elapsed < cfg_.max_pending_command_age_s;
            sample_backlog_guard_active = backlog_guard_engaged;

            if (elapsed < period_floor) {
                // Too soon since last amovel - skip. No log: 60 Hz spam.
                sample_skip_reason = "scheduler";
            } else if (backlog_guard_engaged) {
                sample_skip_reason = "backlog_guard";
            } else if (cfg_.micro_pursuit_enabled &&
                       cfg_.micro_velocity_filter_enabled) {
                // --- PURSUIT + VELOCITY-SMOOTHED step ----------------
                // Stages (logged separately so the CSV pin-points where
                // a discontinuity is introduced):
                //   1. raw_error = desired - last_commanded
                //   2. v_des     = Kp * err clamped to per-axis lin/ang cap
                //   3. v_filt    = LPF(alpha)
                //   4. accel cap (per-axis)        -> accel_limit_applied
                //   5. jerk  cap (per-axis)        -> jerk_limit_applied
                //   6. velocity deadband            -> velocity_deadband_applied
                //   7. integrate to step           -> raw_step_xyz_mm
                //   8. vector-norm cap on XYZ step -> step_norm_clipped
                //   9. emit amovel
                const double dt   = std::max(elapsed, 1e-3);
                const double Kp_l = std::max(cfg_.micro_command_rate_hz, 1.0);
                const double Kp_r = Kp_l;

                // 1. Raw error.
                double err[6] = {
                    desired_target_pose_.x  - last_commanded_target_pose_.x,
                    desired_target_pose_.y  - last_commanded_target_pose_.y,
                    desired_target_pose_.z  - last_commanded_target_pose_.z,
                    desired_target_pose_.rx - last_commanded_target_pose_.rx,
                    desired_target_pose_.ry - last_commanded_target_pose_.ry,
                    desired_target_pose_.rz - last_commanded_target_pose_.rz
                };
                sample_raw_error = std::array<double, 6>{
                    err[0], err[1], err[2], err[3], err[4], err[5]};

                // 2. Desired velocity v_des = Kp * err, clamped to caps.
                double v_des[6] = {
                    clampAbs(Kp_l * err[0], cfg_.micro_lin_vel),
                    clampAbs(Kp_l * err[1], cfg_.micro_lin_vel),
                    clampAbs(Kp_l * err[2], cfg_.micro_lin_vel),
                    clampAbs(Kp_r * err[3], cfg_.micro_ang_vel),
                    clampAbs(Kp_r * err[4], cfg_.micro_ang_vel),
                    clampAbs(Kp_r * err[5], cfg_.micro_ang_vel)
                };
                sample_desired_velocity = std::array<double, 6>{
                    v_des[0], v_des[1], v_des[2], v_des[3], v_des[4], v_des[5]};

                // 3. Low-pass filter.
                const double a = clampAbs(cfg_.micro_velocity_filter_alpha, 1.0);
                double v_filt[6];
                for (int i = 0; i < 6; ++i) {
                    v_filt[i] = a * v_des[i] + (1.0 - a) * filtered_velocity_[i];
                }

                // 4. Acceleration limit. Track whether any axis was clipped.
                const double accel_cap_lin = cfg_.micro_lin_acc * dt;
                const double accel_cap_ang = cfg_.micro_ang_acc * dt;
                for (int i = 0; i < 3; ++i) {
                    const double before = v_filt[i];
                    v_filt[i] = limitDelta(v_filt[i], filtered_velocity_[i],
                                           accel_cap_lin);
                    if (std::abs(v_filt[i] - before) > 1e-9)
                        sample_accel_limit_applied = true;
                }
                for (int i = 3; i < 6; ++i) {
                    const double before = v_filt[i];
                    v_filt[i] = limitDelta(v_filt[i], filtered_velocity_[i],
                                           accel_cap_ang);
                    if (std::abs(v_filt[i] - before) > 1e-9)
                        sample_accel_limit_applied = true;
                }

                // 5. Jerk limit (cap on rate of change of acceleration).
                const double jerk_cap_lin = cfg_.micro_max_jerk_xyz * dt;
                const double jerk_cap_ang = cfg_.micro_max_jerk_rot * dt;
                double accel_new[6];
                for (int i = 0; i < 6; ++i) {
                    accel_new[i] = (v_filt[i] - filtered_velocity_[i]) / dt;
                }
                for (int i = 0; i < 3; ++i) {
                    const double before_a = accel_new[i];
                    accel_new[i] = limitDelta(accel_new[i], last_accel_[i],
                                              jerk_cap_lin);
                    if (std::abs(accel_new[i] - before_a) > 1e-9)
                        sample_jerk_limit_applied = true;
                    v_filt[i] = filtered_velocity_[i] + accel_new[i] * dt;
                }
                for (int i = 3; i < 6; ++i) {
                    const double before_a = accel_new[i];
                    accel_new[i] = limitDelta(accel_new[i], last_accel_[i],
                                              jerk_cap_ang);
                    if (std::abs(accel_new[i] - before_a) > 1e-9)
                        sample_jerk_limit_applied = true;
                    v_filt[i] = filtered_velocity_[i] + accel_new[i] * dt;
                }

                // 6. Velocity deadband: kill micro-dribble.
                const double vbd_lin = cfg_.micro_velocity_deadband_mm_s;
                const double vbd_ang = vbd_lin * 0.1;  // crude scaling for deg/s
                bool any_velocity = false;
                for (int i = 0; i < 3; ++i) {
                    if (std::abs(v_filt[i]) < vbd_lin) {
                        v_filt[i] = 0.0; accel_new[i] = 0.0;
                        sample_velocity_deadband_applied = true;
                    } else any_velocity = true;
                }
                for (int i = 3; i < 6; ++i) {
                    if (std::abs(v_filt[i]) < vbd_ang) {
                        v_filt[i] = 0.0; accel_new[i] = 0.0;
                        sample_velocity_deadband_applied = true;
                    } else any_velocity = true;
                }

                sample_filtered_velocity = std::array<double, 6>{
                    v_filt[0], v_filt[1], v_filt[2],
                    v_filt[3], v_filt[4], v_filt[5]};
                sample_limited_accel = std::array<double, 6>{
                    accel_new[0], accel_new[1], accel_new[2],
                    accel_new[3], accel_new[4], accel_new[5]};

                // Commit state.
                for (int i = 0; i < 6; ++i) {
                    filtered_velocity_[i] = v_filt[i];
                    last_accel_[i]        = accel_new[i];
                }
                last_vel_update_s_ = now_s;

                // Ramp-to-zero completion: stop when velocity drops to
                // the deadband on every axis, or the ramp time budget
                // expires. After that the pursuit pipeline goes quiet
                // and the last in-flight amovel runs to completion.
                if (ramp_to_zero_) {
                    const bool timeout =
                        (now_s - ramp_start_s_) >= cfg_.micro_stop_ramp_time_s;
                    if (!any_velocity || timeout) {
                        LOG_I("Velocity ramp-to-zero complete "
                              "(any_velocity=%d, timeout=%d, elapsed=%.3fs).",
                              any_velocity ? 1 : 0, timeout ? 1 : 0,
                              now_s - ramp_start_s_);
                        event_logger_.event("motion tail complete");
                        ramp_to_zero_                  = false;
                        active_entry_robot_pose_valid_ = false;
                        last_commanded_target_valid_   = false;
                        virtual_target_initialised_    = false;
                        filtered_velocity_.fill(0.0);
                        last_accel_.fill(0.0);
                        last_emitted_step_xyz_mm_ = 0.0;
                        last_emitted_step_rot_deg_= 0.0;
                    }
                }

                if (any_velocity) {
                    // 7. Compute the integrated step (pre-cap).
                    double dx = v_filt[0] * dt;
                    double dy = v_filt[1] * dt;
                    double dz = v_filt[2] * dt;
                    double drx = v_filt[3] * dt;
                    double dry = v_filt[4] * dt;
                    double drz = v_filt[5] * dt;
                    sample_raw_step_xyz_mm  = vec3Norm(dx, dy, dz);
                    sample_raw_step_rot_deg = vec3Norm(drx, dry, drz);

                    // 8. Vector-norm cap. Replaces the previous implicit
                    // per-axis cap which let segments grow to ~sqrt(3)*max.
                    // For the first recovery_soft_commands_remaining_ ticks
                    // after a brief-loss recovery, use the much tighter
                    // recovery caps so the robot eases back into motion.
                    const bool in_soft_recovery =
                        recovery_soft_commands_remaining_ > 0;
                    const double xyz_cap = in_soft_recovery
                        ? cfg_.tracking_loss_max_recovery_step_xyz_mm
                        : cfg_.micro_max_step_xyz_mm;
                    const double rot_cap = in_soft_recovery
                        ? cfg_.tracking_loss_max_recovery_step_rot_deg
                        : cfg_.micro_max_step_rot_deg;
                    const bool clip_xyz = limitVectorNorm3(
                        dx, dy, dz, xyz_cap);
                    const bool clip_rot = limitVectorNorm3(
                        drx, dry, drz, rot_cap);
                    sample_step_norm_clipped = clip_xyz || clip_rot;
                    sample_recovery_step_limited_local = in_soft_recovery;

                    RobotPose pursuit_target = last_commanded_target_pose_;
                    pursuit_target.x  += dx;
                    pursuit_target.y  += dy;
                    pursuit_target.z  += dz;
                    pursuit_target.rx += drx;
                    pursuit_target.ry += dry;
                    pursuit_target.rz += drz;

                    const double seg_norm = vec3Norm(dx, dy, dz);
                    const double rot_norm = vec3Norm(drx, dry, drz);
                    const double radius = cfg_.micro_blending_enabled
                        ? std::min(cfg_.micro_blending_radius_mm,
                                   0.4 * seg_norm)
                        : 0.0;

                    if (cfg_.debug_verbose_robot_commands) {
                        LOG_D("vel-smooth: err=[%.2f %.2f %.2f / %.2f %.2f %.2f] "
                              "v=[%.1f %.1f %.1f / %.2f %.2f %.2f] "
                              "seg=%.2f rot=%.2f r=%.2f clip=%d.",
                              err[0], err[1], err[2], err[3], err[4], err[5],
                              v_filt[0], v_filt[1], v_filt[2],
                              v_filt[3], v_filt[4], v_filt[5],
                              seg_norm, rot_norm, radius,
                              sample_step_norm_clipped ? 1 : 0);
                    }

                    if (robot_.sendCartesianMicroMove(
                            pursuit_target,
                            cfg_.micro_lin_vel, cfg_.micro_ang_vel,
                            cfg_.micro_lin_acc, cfg_.micro_ang_acc,
                            radius)) {
                        sample_command_interval_ms =
                            (last_command_sent_s_ > 0.0)
                                ? (now_s - last_command_sent_s_) * 1000.0
                                : 0.0;
                        last_commanded_target_pose_ = pursuit_target;
                        last_command_sent_s_        = now_s;
                        sample_command_sent     = true;
                        sample_commanded_target = pursuit_target.toArray();
                        sample_step_xyz = seg_norm;
                        sample_step_rot = rot_norm;
                        last_emitted_step_xyz_mm_  = seg_norm;
                        last_emitted_step_rot_deg_ = rot_norm;
                        if (recovery_soft_commands_remaining_ > 0)
                            --recovery_soft_commands_remaining_;
                    } else {
                        LOG_W("Pursuit-velocity micro-move refused: %s",
                              robot_.lastError().c_str());
                        sample_skip_reason = "robot_refused";
                        event_logger_.event(std::string("Command skipped: robot refused (") +
                                            robot_.lastError() + ")");
                    }
                } else {
                    sample_skip_reason = "velocity_deadband";
                }
            } else if (cfg_.micro_pursuit_enabled) {
                // --- PURSUIT step (position-step path, no velocity filter) ---
                // Vector-norm version: compute the full error vector,
                // skip if its norm is inside the arrival band, otherwise
                // advance toward it by up to max_step (Euclidean) - never
                // per-axis. Avoids the sqrt(3)*max bug.
                double ex = desired_target_pose_.x  - last_commanded_target_pose_.x;
                double ey = desired_target_pose_.y  - last_commanded_target_pose_.y;
                double ez = desired_target_pose_.z  - last_commanded_target_pose_.z;
                double erx = desired_target_pose_.rx - last_commanded_target_pose_.rx;
                double ery = desired_target_pose_.ry - last_commanded_target_pose_.ry;
                double erz = desired_target_pose_.rz - last_commanded_target_pose_.rz;

                sample_raw_error = std::array<double, 6>{
                    ex, ey, ez, erx, ery, erz};

                const double err_xyz_norm = vec3Norm(ex,  ey,  ez);
                const double err_rot_norm = vec3Norm(erx, ery, erz);
                sample_raw_step_xyz_mm  = err_xyz_norm;
                sample_raw_step_rot_deg = err_rot_norm;

                // Soft-cap during the recovery window: tighter caps for
                // the first N commands so the first move out of hold is
                // gentle.
                const bool in_soft_recovery =
                    recovery_soft_commands_remaining_ > 0;
                sample_recovery_step_limited_local = in_soft_recovery;
                const double cap_xyz = in_soft_recovery
                    ? cfg_.tracking_loss_max_recovery_step_xyz_mm
                    : cfg_.micro_max_step_xyz_mm;
                const double cap_rot = in_soft_recovery
                    ? cfg_.tracking_loss_max_recovery_step_rot_deg
                    : cfg_.micro_max_step_rot_deg;

                double sx = 0.0, sy = 0.0, sz = 0.0;
                double srx = 0.0, sry = 0.0, srz = 0.0;
                if (err_xyz_norm > cfg_.micro_arrival_band_xyz_mm) {
                    const double step_xyz = std::max(
                        std::min(err_xyz_norm, cap_xyz),
                        std::min(cfg_.micro_min_step_xyz_mm, err_xyz_norm));
                    const double k = err_xyz_norm > 1e-9
                        ? step_xyz / err_xyz_norm : 0.0;
                    sx = ex * k; sy = ey * k; sz = ez * k;
                    if (step_xyz >= cap_xyz - 1e-9 && err_xyz_norm > cap_xyz)
                        sample_step_norm_clipped = true;
                }
                if (err_rot_norm > cfg_.micro_arrival_band_rot_deg) {
                    const double step_rot = std::max(
                        std::min(err_rot_norm, cap_rot),
                        std::min(cfg_.micro_min_step_rot_deg, err_rot_norm));
                    const double k = err_rot_norm > 1e-9
                        ? step_rot / err_rot_norm : 0.0;
                    srx = erx * k; sry = ery * k; srz = erz * k;
                    if (step_rot >= cap_rot - 1e-9 && err_rot_norm > cap_rot)
                        sample_step_norm_clipped = true;
                }

                const double seg_norm = vec3Norm(sx, sy, sz);
                const double rot_norm = vec3Norm(srx, sry, srz);
                const bool any_step = seg_norm > 1e-9 || rot_norm > 1e-9;

                if (any_step) {
                    RobotPose pursuit_target = last_commanded_target_pose_;
                    pursuit_target.x  += sx;
                    pursuit_target.y  += sy;
                    pursuit_target.z  += sz;
                    pursuit_target.rx += srx;
                    pursuit_target.ry += sry;
                    pursuit_target.rz += srz;

                    const double radius = cfg_.micro_blending_enabled
                        ? std::min(cfg_.micro_blending_radius_mm,
                                   0.4 * seg_norm)
                        : 0.0;

                    if (robot_.sendCartesianMicroMove(
                            pursuit_target,
                            cfg_.micro_lin_vel, cfg_.micro_ang_vel,
                            cfg_.micro_lin_acc, cfg_.micro_ang_acc,
                            radius)) {
                        sample_command_interval_ms =
                            (last_command_sent_s_ > 0.0)
                                ? (now_s - last_command_sent_s_) * 1000.0
                                : 0.0;
                        last_commanded_target_pose_ = pursuit_target;
                        last_command_sent_s_        = now_s;
                        sample_command_sent     = true;
                        sample_commanded_target = pursuit_target.toArray();
                        sample_step_xyz = seg_norm;
                        sample_step_rot = rot_norm;
                        last_emitted_step_xyz_mm_  = seg_norm;
                        last_emitted_step_rot_deg_ = rot_norm;
                        if (recovery_soft_commands_remaining_ > 0)
                            --recovery_soft_commands_remaining_;
                    } else {
                        LOG_W("Pursuit micro-move refused by adapter: %s",
                              robot_.lastError().c_str());
                        sample_skip_reason = "robot_refused";
                        event_logger_.event(std::string("Command skipped: robot refused (") +
                                            robot_.lastError() + ")");
                    }
                } else {
                    sample_skip_reason = "arrival_band";
                }
            } else {
                // --- LEGACY incremental mode ---------------------------
                std::array<double, 6> twist = {
                    cmd.linear_velocity[0],  cmd.linear_velocity[1],  cmd.linear_velocity[2],
                    cmd.angular_velocity[0], cmd.angular_velocity[1], cmd.angular_velocity[2]
                };
                guard_.clampSpeed(twist);

                double dx  = twist[0] * elapsed;
                double dy  = twist[1] * elapsed;
                double dz  = twist[2] * elapsed;
                double drx = twist[3] * elapsed;
                double dry = twist[4] * elapsed;
                double drz = twist[5] * elapsed;

                sample_raw_step_xyz_mm  = vec3Norm(dx, dy, dz);
                sample_raw_step_rot_deg = vec3Norm(drx, dry, drz);

                // Vector-norm cap (replaces the per-axis bound() that
                // produced ~sqrt(3)*cap segments under diagonal motion).
                const bool clip_xyz = limitVectorNorm3(
                    dx, dy, dz, cfg_.micro_max_delta_xyz_mm);
                const bool clip_rot = limitVectorNorm3(
                    drx, dry, drz, cfg_.micro_max_delta_rot_deg);
                sample_step_norm_clipped = clip_xyz || clip_rot;

                const bool meaningful =
                    vec3Norm(dx, dy, dz)  > cfg_.micro_deadband_mm ||
                    vec3Norm(drx, dry, drz) > cfg_.micro_deadband_deg;

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
                        sample_command_interval_ms =
                            (last_command_sent_s_ > 0.0)
                                ? (now_s - last_command_sent_s_) * 1000.0
                                : 0.0;
                        last_command_sent_s_ = now_s;
                        sample_command_sent     = true;
                        sample_commanded_target = virtual_target_pose_.toArray();
                        sample_step_xyz = vec3Norm(dx, dy, dz);
                        sample_step_rot = vec3Norm(drx, dry, drz);
                        last_emitted_step_xyz_mm_  = sample_step_xyz;
                        last_emitted_step_rot_deg_ = sample_step_rot;
                    } else {
                        LOG_W("Micro-move refused by adapter: %s",
                              robot_.lastError().c_str());
                        sample_skip_reason = "robot_refused";
                        event_logger_.event(std::string("Command skipped: robot refused (") +
                                            robot_.lastError() + ")");
                    }
                } else {
                    sample_skip_reason = "deadband";
                }
            }
        }
        fault_emergency_issued_ = false;
    } else if (prev_active) {
        // Falling edge active -> passive. Start the velocity ramp-to-
        // zero: keep the controller running for a short tail so the
        // filtered velocity decays under the configured accel / jerk
        // caps, instead of disappearing in one step. Each tail tick
        // still issues amovel commands - NO stopMotion(), NO speedl(0),
        // NO emergencyStop, NO mwait. The ramp branch is entered on
        // the NEXT tick via (ramp_to_zero_ && !cur_active).
        if (cfg_.micro_velocity_filter_enabled &&
            active_entry_robot_pose_valid_) {
            ramp_to_zero_ = true;
            ramp_start_s_ = now_s;
            LOG_I("State %s -> %s : starting velocity ramp-to-zero "
                  "(~%.2fs).",
                  stateName(prev_state_), stateName(cur_state),
                  cfg_.micro_stop_ramp_time_s);
        } else {
            // Velocity smoothing disabled - keep the legacy behaviour:
            // stop issuing, let the last amovel run to completion.
            LOG_I("State %s -> %s : ceasing micro-motion stream.",
                  stateName(prev_state_), stateName(cur_state));
            virtual_target_initialised_    = false;
            active_entry_robot_pose_valid_ = false;
            last_commanded_target_valid_   = false;
        }
        fault_emergency_issued_        = false;
    } else {
        // Passive -> passive (stable). Do NOT touch the robot.
        fault_emergency_issued_ = critical ? fault_emergency_issued_ : false;
    }

    if (cur_state != prev_state_) {
        event_logger_.event(std::string("State: ") + stateName(prev_state_) +
                            " -> " + stateName(cur_state),
                            cfg_.runtime_tuning_print_changes_to_console);
    }
    prev_state_ = cur_state;

    if (cmd.openGripper) {
        event_logger_.event("Gripper IMPULSE: open");
        gripper_.open();
    }
    if (cmd.closeGripper) {
        event_logger_.event("Gripper IMPULSE: close");
        gripper_.close();
    }

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

    // --- Motion CSV sample ------------------------------------------------
    // Build a record only if logging is on AND (we want to log everywhere
    // OR we're in an active control phase). Cost when off: a single bool
    // branch.
    if (motion_logger_.isOpen()) {
        const bool log_this_tick =
            !cfg_.logging_only_when_active || cur_active || ramp_to_zero_;
        if (log_this_tick) {
            MotionLogSample s;
            s.timestamp_s   = now_s;
            s.dt_ms         = dt_ms;
            s.tracking_valid = tracking_valid_now;
            s.deadman_active = btn;
            s.control_mode   = stateName(cur_state);
            s.command_sent   = sample_command_sent;
            s.command_skip_reason = sample_skip_reason;

            if (frame.right) {
                s.hand_raw_pos = frame.right->palm_position;
                // Cheap raw orientation estimate (extrinsic XYZ deg) from
                // direction + normal so the CSV has a comparable column
                // to hand_filtered_rot.
                const auto& d = frame.right->palm_direction;
                const auto& n = frame.right->palm_normal;
                double yaw   = std::atan2(d[0], -d[2]) * kRad2Deg;
                double pitch = (std::abs(d[0]) + std::abs(d[1]) + std::abs(d[2]) > 1e-6)
                    ? std::asin(clamp(d[1], -1.0, 1.0)) * kRad2Deg : 0.0;
                double roll  = std::atan2(n[0], -n[1]) * kRad2Deg;
                s.hand_raw_rot = std::array<double, 3>{pitch, yaw, roll};
            }
            if (interpreter_.smootherPrimed()) {
                s.hand_filtered_pos = interpreter_.smoothedRightPosition();
                const auto& d = interpreter_.smoothedRightDirection();
                const auto& n = interpreter_.smoothedRightNormal();
                double yaw   = std::atan2(d[0], -d[2]) * kRad2Deg;
                double pitch = std::asin(clamp(d[1], -1.0, 1.0)) * kRad2Deg;
                double roll  = std::atan2(n[0], -n[1]) * kRad2Deg;
                s.hand_filtered_rot = std::array<double, 3>{pitch, yaw, roll};
            }
            s.hand_delta_pos = report.rightDeltaPosition;
            s.hand_delta_rot = report.rightDeltaOrientation;

            s.desired_target  = sample_desired_target;
            s.last_commanded  = sample_last_commanded;
            s.commanded       = sample_commanded_target;
            if (cfg_.logging_include_actual_pose) {
                s.actual_robot = last_pose_for_ui_.toArray();
            }

            if (s.desired_target && s.last_commanded) {
                const auto& d = *s.desired_target;
                const auto& l = *s.last_commanded;
                s.position_error_mm = std::sqrt(
                    (d[0]-l[0])*(d[0]-l[0]) +
                    (d[1]-l[1])*(d[1]-l[1]) +
                    (d[2]-l[2])*(d[2]-l[2]));
                s.rotation_error_deg = std::sqrt(
                    (d[3]-l[3])*(d[3]-l[3]) +
                    (d[4]-l[4])*(d[4]-l[4]) +
                    (d[5]-l[5])*(d[5]-l[5]));
            }
            s.commanded_step_xyz_mm  = sample_step_xyz;
            s.commanded_step_rot_deg = sample_step_rot;

            s.micro_command_rate_hz    = cfg_.micro_command_rate_hz;
            s.micro_min_period_s       = cfg_.micro_min_period_s;
            s.micro_lin_vel            = cfg_.micro_lin_vel;
            s.micro_lin_acc            = cfg_.micro_lin_acc;
            s.micro_ang_vel            = cfg_.micro_ang_vel;
            s.micro_ang_acc            = cfg_.micro_ang_acc;
            s.micro_blending_enabled   = cfg_.micro_blending_enabled;
            s.micro_blending_radius_mm = cfg_.micro_blending_radius_mm;
            s.micro_pursuit_enabled    = cfg_.micro_pursuit_enabled;
            s.micro_hand_to_robot_ratio= cfg_.micro_hand_to_robot_ratio;
            s.motion_position_scale    = cfg_.position_scale;
            s.motion_orientation_scale = cfg_.orientation_scale;
            s.motion_smoothing_alpha   = cfg_.smoothing_alpha;

            // Extended diagnostics.
            s.command_interval_ms              = sample_command_interval_ms;
            s.scheduler_elapsed_ms             = sample_scheduler_elapsed_ms;
            s.previous_motion_estimated_time_ms = sample_prev_motion_est_ms;
            s.backlog_guard_active             = sample_backlog_guard_active;
            const double target_dt_ms = 1000.0 / static_cast<double>(cfg_.loop_rate_hz);
            s.loop_overrun_ms = std::max(0.0, dt_ms - target_dt_ms);
            s.cached_actual_pose_age_ms =
                (last_pose_poll_s_ > 0.0) ? (now_s - last_pose_poll_s_) * 1000.0 : 0.0;
            // Cached pose is "live" only when we're NOT in active streaming
            // (active path never polls getCurrentPose, alarm 5.7056).
            s.actual_pose_live = !cur_active && !ramp_to_zero_;

            s.raw_error         = sample_raw_error;
            s.desired_velocity  = sample_desired_velocity;
            s.filtered_velocity = sample_filtered_velocity;
            s.limited_accel     = sample_limited_accel;
            s.raw_step_xyz_mm   = sample_raw_step_xyz_mm;
            s.limited_step_xyz_mm  = sample_step_xyz;
            s.raw_step_rot_deg  = sample_raw_step_rot_deg;
            s.limited_step_rot_deg = sample_step_rot;
            s.velocity_deadband_applied = sample_velocity_deadband_applied;
            s.jerk_limit_applied        = sample_jerk_limit_applied;
            s.accel_limit_applied       = sample_accel_limit_applied;
            s.step_norm_clipped         = sample_step_norm_clipped;
            s.tracking_stable_age_ms    = tracking_age_s * 1000.0;

            // Brief tracking-loss tolerance diagnostics.
            s.tracking_loss_tolerance_enabled = cfg_.tracking_loss_tolerance_enabled;
            s.tracking_recent             = tracking_recent;
            s.brief_tracking_loss_active  = brief_loss_active;
            s.hard_tracking_loss          = hard_tracking_loss;
            s.tracking_loss_duration_ms   =
                (last_valid_tracking_s_ < 0.0)
                    ? 0.0 : tracking_loss_age_s * 1000.0;
            s.tracking_recovery_stable_ms =
                (tracking_hold_active_ && tracking_recent_again_s_ > 0.0)
                    ? (now_s - tracking_recent_again_s_) * 1000.0
                    : 0.0;
            s.tracking_hold_active        = tracking_hold_active_;
            s.reanchor_performed          = reanchor_performed;
            s.recovery_step_limited       = sample_recovery_step_limited_local
                                            && sample_command_sent;

            motion_logger_.append(s);
        }
    }

    if (sample_command_sent)             ++sent_in_window_;
    else if (sample_skip_reason[0] != '\0') ++skipped_in_window_;

    emitConsoleSummary(now_s, cur_active);

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

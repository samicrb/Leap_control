#pragma once

#include "config/Config.hpp"
#include "config/RuntimeConfigReloader.hpp"
#include "gesture/GestureInterpreter.hpp"
#include "gripper/IGripperController.hpp"
#include "input/IExternalButton.hpp"
#include "logging/EventLogger.hpp"
#include "logging/MotionLogger.hpp"
#include "robot/IRobotController.hpp"
#include "robot/WorkspaceGuard.hpp"
#include "sensor/ILeapSource.hpp"
#include "state/StateMachine.hpp"
#include "ui/ConsoleUI.hpp"

#include <atomic>
#include <array>
#include <memory>
#include <string>

namespace dgd {

// Application owns the demo lifecycle. Dependencies are injected so the
// whole thing can be driven by mocks in a test binary.
class Application {
public:
    // The Config is taken by non-const reference so the runtime reloader
    // can mutate whitelisted fields in place from inside the tick loop.
    // All other consumers still see a const reference and never observe
    // partial writes (changes are committed once per poll, between two
    // ticks).
    Application(Config&            cfg,
                ILeapSource&       sensor,
                IRobotController&  robot,
                IGripperController& gripper,
                IExternalButton&   button,
                std::string        config_path = "demo_config.ini");

    // Prepares everything for the first tick. Moves the robot home.
    bool initialise();

    // Blocking run loop. Exits when stop() is called or the button
    // source signals shutdown.
    int run();

    // Thread-safe shutdown request.
    void stop();

private:
    void tick(double now_s);
    double loopPeriod() const { return 1.0 / static_cast<double>(cfg_.loop_rate_hz); }
    void rebindLoggingIfNeeded();
    void emitConsoleSummary(double now_s, bool cur_active);

    Config&            cfg_;
    ILeapSource&       sensor_;
    IRobotController&  robot_;
    IGripperController& gripper_;
    IExternalButton&   button_;

    GestureInterpreter interpreter_;
    StateMachine       sm_;
    WorkspaceGuard     guard_;
    ConsoleUI          ui_;

    // Diagnostic + tuning sidecar. None of these touch the robot path
    // when their respective config flag is false.
    MotionLogger             motion_logger_;
    EventLogger              event_logger_;
    RuntimeConfigReloader    reloader_;
    std::string              config_path_;
    bool                     logging_was_enabled_     = false;
    bool                     motion_csv_was_enabled_  = false;
    bool                     event_log_was_enabled_   = false;
    std::string              prior_log_directory_;
    std::string              prior_experiment_name_;
    double                   last_tick_s_             = 0.0;
    double                   last_summary_s_          = 0.0;
    int                      sent_in_window_          = 0;
    int                      skipped_in_window_       = 0;
    double                   summary_window_start_s_  = 0.0;

    HandFrame          last_frame_{};
    std::atomic<bool>  running_{false};
    // Latches whether we already fired the one-shot emergencyStop() for
    // the current Fault entry. Reset whenever we leave the Fault state.
    bool               fault_emergency_issued_ = false;
    // Previous tick's DemoState; used to detect active->passive and
    // passive->active transitions so we only send stopMotion() once on
    // the falling edge (never at the 60 Hz loop rate while sitting in
    // a stable passive state - this is what was leading to alarm 5.7056
    // "Standstill status violated").
    DemoState          prev_state_ = DemoState::Idle;
    // UI pose cache. We MUST NOT call robot_.getCurrentPose() during
    // active streaming (PositionControl / OrientationControl) - that
    // call resolves to a non-motion DRFL command
    // (CONTROL_CHECK_CURRENT_TASK_POSITION) and interleaving it with
    // speedl() reliably trips alarm 5.7056. Instead we cache the pose
    // and refresh it at low frequency only while the streaming pipe
    // is quiet.
    RobotPose          last_pose_for_ui_{};
    double             last_pose_poll_s_ = 0.0;

    // --- Micro-motion supervisor state ---
    //
    // The Leap hand drives an absolute desired target relative to the
    // robot pose captured ONCE on entering active mode (the only
    // getCurrentPose call allowed in the active path). The pursuit
    // controller then advances last_commanded_target_pose_ toward
    // desired_target_pose_ via bounded amovel steps. Blending across
    // consecutive segments is on by default for visual continuity.
    //
    // The legacy incremental path (virtual_target_pose_) is kept and
    // selected via cfg_.micro_pursuit_enabled = false.
    RobotPose             virtual_target_pose_{};        // legacy path
    bool                  virtual_target_initialised_ = false;
    double                last_command_sent_s_ = 0.0;

    RobotPose             active_entry_robot_pose_{};
    bool                  active_entry_robot_pose_valid_ = false;
    RobotPose             desired_target_pose_{};
    RobotPose             last_commanded_target_pose_{};
    bool                  last_commanded_target_valid_ = false;

    // Velocity smoothing layer state. v[0..2] are linear (mm/s), v[3..5]
    // are angular (deg/s). The controller integrates filtered_velocity_
    // into last_commanded_target_pose_ each tick. ramp_to_zero_ keeps
    // the controller running for a brief tail after active mode ends,
    // so the robot never sees an instantaneous velocity step to zero.
    std::array<double, 6> filtered_velocity_ {0, 0, 0, 0, 0, 0};
    std::array<double, 6> last_accel_        {0, 0, 0, 0, 0, 0};
    double                last_vel_update_s_ = 0.0;
    bool                  ramp_to_zero_      = false;
    double                ramp_start_s_      = 0.0;

    // --- Tracking-stability gate ---
    // Once tracking goes invalid (sensor lost OR no fresh frame OR right
    // hand below confidence) we freeze the pursuit pipeline and refuse
    // to send amovel commands. On recovery we require the tracking to
    // remain valid for cfg_.micro_tracking_recovery_time_s before re-
    // arming, which prevents the POSITION -> RECENTER -> FAULT chatter
    // observed in field logs.
    bool                  tracking_valid_last_ = false;
    double                tracking_valid_since_s_ = -1.0;
    bool                  pursuit_armed_      = false;

    // --- Backlog guard state ---
    // Stores the magnitude of the last emitted segment so the scheduler
    // can estimate t_est = max(step_xyz / v_lin, step_rot / v_ang) and
    // wait min_motion_completion_ratio * t_est before queueing the next.
    double                last_emitted_step_xyz_mm_ = 0.0;
    double                last_emitted_step_rot_deg_= 0.0;
};

} // namespace dgd

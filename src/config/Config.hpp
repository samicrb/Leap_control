#pragma once

// Strongly-typed config snapshot loaded from config/demo_config.ini.
// Plain key=value format, no external dependencies.

#include <string>
#include <unordered_map>

namespace dgd {

struct Config {
    // --- robot ---
    std::string robot_ip       = "192.168.1.25";
    int         robot_port     = 12345;
    std::string robot_model    = "M1013";
    double      connect_timeout_s = 5.0;
    // Skip the blocking movel-to-safe-pose at startup. Useful when the
    // safe pose triggers a SAFE_STOP from a controller-side safety zone /
    // collision check: the demo then keeps whatever pose the robot is
    // already in and runs gestures from there.
    bool        skip_move_home = false;
    // Positive-named, demo-safe master switches for the home approach.
    // Defaults FALSE so a fresh deployment never moves the robot on its
    // own. Effective rule (Application::initialise):
    //   move_home_now = return_home_on_start && !skip_move_home
    // This preserves backward compat for INIs that only set skip_move_home.
    bool        return_home_on_start    = false;
    bool        return_home_on_shutdown = false;
    // Collision detection sensitivity (0..100). 0 disables collision
    // detection entirely - the only reliable bring-up value when the
    // controller-side payload / TCP info isn't known yet and gentle
    // motions keep tripping a phantom collision SAFE_STOP. Re-enable
    // (50-75) once payload and TCP are configured for production.
    int         collision_sensitivity = 0;
    // Singularity handling mode for movel / speedl. Maps directly onto
    // the Doosan SINGULARITY_AVOIDANCE enum:
    //   0 = AVOID  - blend through the singularity automatically
    //   1 = STOP   - default; halt motion on entering a singular region
    //                (this is what trips alarm 3205 -> SAFE_OFF 7056)
    //   2 = VEL    - reduce velocity in the singular region
    // AVOID (0) is the demo-friendly choice for any path that may graze
    // a wrist singularity (e.g. ry near 0/180 in ZYZ' Euler).
    int         singularity_handling = 0;
    // If true, the safe-pose approach uses movejx (joint-space interp to
    // a Cartesian target) instead of movel (Cartesian linear). movejx
    // ignores Cartesian path singularities so it's the robust choice for
    // the initial home approach.
    bool        home_use_movejx = true;
    // If true, engage() auto-fires CONTROL_RESET_SAFET_STOP /
    // CONTROL_RESET_SAFET_OFF whenever the robot is found latched in one
    // of those states at startup. A SAFE_STOP from a previous run
    // remains latched on the controller and looks identical to a
    // "mastering lost" condition - servo_on refuses to re-engage.
    // DRFL CANNOT bypass an actual mastering loss (firmware-level safety
    // check); if state stays in RECOVERY after the reset attempts, the
    // pendant menu Setting -> Robot -> Mastering is the only recovery.
    bool        auto_reset_safety = true;

    // --- loop ---
    // Profile aligned with the micro-pursuit segment dynamics:
    //   typical segment length ~ v^2 / a  =>  120^2 / 650 ~ 22 mm
    // so max_step_xyz_mm and per-segment caps match these caps.
    int    loop_rate_hz    = 60;
    double max_lin_speed   = 120.0;
    double max_ang_speed   = 28.0;
    double max_lin_accel   = 650.0;
    double max_ang_accel   = 160.0;

    // --- workspace envelope (mm, deg) ---
    // When ws_enabled = false the box and orientation cone are NOT enforced
    // (only hard speed/accel caps still apply). Useful for initial bring-up
    // before the cell has been characterised.
    bool   ws_enabled = false;
    double ws_x_min = -300.0, ws_x_max = 400.0;
    double ws_y_min = 200.0,  ws_y_max = 600.0;
    double ws_z_min = 150.0,  ws_z_max = 600.0;
    double ws_rx_range = 45.0, ws_ry_range = 45.0, ws_rz_range = 90.0;

    // --- safe pose (robot BASE frame; Doosan ZYZ' Euler) ---
    // safe_rx/ry/rz match the pendant display order [W, P, R]:
    //   safe_rx == W  : rotation about BASE Z
    //   safe_ry == P  : rotation about Y'   (intermediate)
    //   safe_rz == R  : rotation about Z''  (final)
    double safe_x  =  55.0, safe_y = 400.0, safe_z = 375.0;
    double safe_rx =  33.0, safe_ry =  96.0, safe_rz = 110.0;

    // --- gesture ---
    double grab_closed_threshold = 0.85;
    double grab_open_threshold   = 0.30;
    double min_confidence        = 0.3;
    double hand_loss_timeout_s   = 0.15;
    double posture_hold_s        = 0.10;

    // --- motion mapping ---
    // Profile tuned for the micro-pursuit controller. position_scale is
    // the gain applied by the state machine to the raw hand displacement
    // (the pursuit controller divides back by it to recover hand_delta,
    // then re-applies micro_hand_to_robot_ratio).
    double position_scale    = 0.90;
    double orientation_scale = 0.45;
    double position_deadzone_mm = 1.5;
    double orientation_deadzone_deg = 1.5;
    double smoothing_alpha   = 0.14;
    int    sign_x = 1, sign_y = 1, sign_z = 1;
    int    sign_rx = 1, sign_ry = 1, sign_rz = 1;

    // --- gripper (generic tool-DO end-effector) ---
    double gripper_open_mm    = 170.0;
    double gripper_close_mm   = 50.0;
    double gripper_neutral_min_mm = 80.0;
    double gripper_neutral_max_mm = 140.0;
    double gripper_facing_dot_max = -0.5;
    double gripper_cooldown_s = 0.8;
    double gripper_gesture_hold_s = 0.25;
    int    gripper_open_do_index  = 2;
    int    gripper_close_do_index = 1;
    // Duration the Tool DO is held HIGH during pulseDO() (ms). Clamped
    // to a 50 ms floor at use site so misconfiguration cannot stop the
    // SoftHand from latching. Default 1000 ms suits the qb SoftHand
    // Industry; lower values (200..300) are fine for fast valves.
    int    gripper_pulse_high_ms       = 1000;
    // Optional bring-up helper: when true, Application::initialise()
    // fires a single pulse on gripper_test_pulse_index after the
    // gripper connects, independent of any Leap gesture. Useful to
    // verify O01 / O02 with a multimeter or directly on the SoftHand.
    bool   gripper_test_pulse_on_start = false;
    int    gripper_test_pulse_index    = 1;

    // --- button ---
    std::string button_mode = "keyboard";
    std::string button_key  = "SPACE";

    // --- UI ---
    bool ui_clear_each_tick = true;
    bool ui_show_hand_debug = false;

    // --- logging ---
    std::string log_level = "INFO";
    std::string log_file  = "doosan_gesture_demo.log";

    // --- dry-run flags ---
    bool dryrun_robot   = false;
    bool dryrun_gripper = false;

    // --- micro-motion supervisor (replaces continuous speedl streaming) ---
    // The active control loop schedules discrete amovel commands. With
    // pursuit_enabled (default) the operator's hand defines an absolute
    // desired target relative to the pose captured on active-mode entry,
    // and the robot follows it via bounded lookahead steps. Without
    // pursuit, the legacy incremental integrator is used.
    double micro_command_rate_hz       = 10.0;   // robot command issue rate
    double micro_min_period_s          = 0.10;   // hard lower bound between commands
    double micro_max_delta_xyz_mm      = 22.0;   // legacy incremental cap
    double micro_max_delta_rot_deg     = 4.0;
    double micro_deadband_mm           = 0.5;
    double micro_deadband_deg          = 0.3;
    // amovel motion profile.
    double micro_lin_vel               = 120.0;  // mm/s
    double micro_ang_vel               = 28.0;   // deg/s
    double micro_lin_acc               = 650.0;  // mm/s^2
    double micro_ang_acc               = 160.0;  // deg/s^2
    // Blending. Controls smoothness across consecutive amovel segments.
    // Disable to revert exactly to the previous non-blended behaviour.
    bool        micro_blending_enabled   = true;
    double      micro_blending_radius_mm = 8.0;
    std::string micro_blending_type      = "duplicate";  // duplicate | override
    // Pursuit / lookahead controller. Smooths the perceived motion by
    // tracking an absolute desired target (ratio * hand_displacement
    // from active-mode entry) with bounded pursuit steps.
    bool   micro_pursuit_enabled      = true;
    double micro_hand_to_robot_ratio  = 0.60;
    double micro_min_step_xyz_mm      = 5.0;
    double micro_max_step_xyz_mm      = 22.0;
    double micro_min_step_rot_deg     = 0.8;
    double micro_max_step_rot_deg     = 4.0;
    double micro_arrival_band_xyz_mm  = 2.0;
    double micro_arrival_band_rot_deg = 0.5;

    // --- Velocity smoothing layer (sits on top of the pursuit controller) ---
    // When enabled, the active control loop no longer commands a position
    // step directly: it derives a desired Cartesian VELOCITY from the
    // pose error, low-pass filters it, then applies hard acceleration and
    // jerk limits before integrating to the next target. The result is a
    // continuous velocity profile that no longer accelerates/decelerates
    // visibly between segments.
    //
    // Disable to revert to the pure position-step pursuit controller.
    // DEFAULT FALSE in this branch: with the vector-norm cap, the
    // step-based pursuit produces cleanly bounded motions on its own,
    // and the field demo currently prioritises reliable motion over
    // velocity-smoothed feel.
    bool   micro_velocity_filter_enabled = false;
    // Low-pass filter alpha on the desired velocity (1 = no filtering,
    // 0 = infinite filtering / frozen). Lower = smoother but laggier.
    double micro_velocity_filter_alpha   = 0.35;
    // Hard jerk caps (mm/s^3 and deg/s^3). Smaller = silkier accel
    // transitions, larger = snappier.
    double micro_max_jerk_xyz            = 3000.0;
    double micro_max_jerk_rot            = 800.0;
    // Below this band, treat the filtered velocity as zero and skip the
    // amovel - avoids dribbling commands when the hand is held still.
    double micro_velocity_deadband_mm_s  = 1.5;
    // When leaving active mode (hand lost, deadman released, etc.) the
    // controller drives the desired velocity to zero over this duration
    // using the same accel / jerk limits. NO speedl, NO stopMotion - the
    // tail ticks are still amovels with shrinking velocity targets.
    double micro_stop_ramp_time_s        = 0.18;

    // --- Tracking stability gate (pursuit re-arm guard) -------------------
    // After tracking is lost, refuse to update the pursuit target / send
    // amovel commands until tracking has been continuously valid for at
    // least this duration. Prevents the POSITION -> RECENTER -> FAULT
    // oscillation seen on borderline confidence values.
    double micro_tracking_recovery_time_s = 0.30;

    // --- Brief tracking-loss tolerance ------------------------------------
    // When enabled, short Leap dropouts (typical for an open-house demo
    // where the operator moves their hand near the edge of the working
    // volume) do NOT push the SM into FAULT. Instead the Application
    // enters an internal "tracking_hold" - target frozen, velocity ramped
    // to zero - and re-anchors the hand reference on recovery so the
    // robot does not jump.
    bool   tracking_loss_tolerance_enabled         = true;
    // Duration past last fresh valid frame before tracking is considered
    // temporarily lost. Single non-fresh ticks (sub-frame jitter at the
    // 60 Hz loop) should not invalidate tracking.
    double tracking_loss_frame_timeout_s           = 0.15;
    // Max duration of tolerated loss. Beyond this we escalate to the
    // existing strict behaviour (SM FAULT, lift-and-show recovery).
    double tracking_loss_brief_timeout_s           = 0.50;
    // How long tracking must be valid again before motion resumes after
    // a brief-loss hold.
    double tracking_loss_recovery_stability_s      = 0.10;
    // Behavioural toggles.
    bool   tracking_loss_freeze_target_on_loss     = true;
    bool   tracking_loss_reanchor_on_recovery      = true;
    bool   tracking_loss_reset_velocity_on_recovery= true;
    bool   tracking_loss_ramp_to_zero_on_loss      = true;
    bool   tracking_loss_fault_after_timeout       = true;
    bool   tracking_loss_require_recenter_after_brief_loss = false;
    // First N amovel commands after recovery use these tighter step caps
    // (Euclidean norm) so the robot resumes from zero/near-zero softly.
    double tracking_loss_max_recovery_step_xyz_mm  = 2.0;
    double tracking_loss_max_recovery_step_rot_deg = 0.5;
    int    tracking_loss_recovery_soft_commands    = 3;

    // --- Command-backlog guard -------------------------------------------
    // The amovel scheduler estimates how long the previously emitted
    // segment will take to execute (max(step_xyz/v_lin, step_rot/v_ang))
    // and refuses to enqueue the next segment until at least
    // min_motion_completion_ratio of that estimate has elapsed. This
    // stops the controller from accumulating a long queue of micro-
    // segments that the robot then catches up to in bursts.
    bool   prevent_command_backlog     = true;
    double min_motion_completion_ratio = 0.60;
    // Hard upper bound: if the previous segment is somehow still flagged
    // pending after this many seconds, force-release the guard so the
    // pursuit doesn't stall forever.
    double max_pending_command_age_s   = 0.35;

    // --- Tool (TCP + Tool Weight) --------------------------------------
    // The TCP and Tool Weight MUST already exist on the Doosan controller
    // (pendant -> Setting -> Robot -> Tool / TCP). The Application only
    // APPLIES the configured names via DRFL set_tcp() / set_tool(). It
    // NEVER creates entries, never calls add_tcp / add_tool, and never
    // modifies the controller-side definitions. Empty name = skip.
    bool        tool_apply_on_start   = true;
    std::string tool_tcp_name         = "";
    std::string tool_tool_weight_name = "";

    // --- Gripper (optional, vendor-agnostic) -----------------------------
    // gripper_enabled = false: the demo behaves exactly as if no gripper
    // were present. NoopGripper is used; no SDK / library is required.
    // gripper_enabled = true:  the configured backend is constructed.
    //   type    : informative tag (e.g. "qb_softhand_industry", "tool_io")
    //   backend : selects the actual implementation
    //             "none"   - NoopGripper (logging-only, no I/O)
    //             "tool_io"- ToolIoGripperController (existing DRFL tool DO)
    // The remaining qb_* parameters are stubs for a future UDP/API
    // backend; they are loaded today so a fresh INI keeps the slots.
    bool        gripper_enabled         = false;
    std::string gripper_type            = "qb_softhand_industry";
    std::string gripper_backend         = "none";
    std::string gripper_ip              = "192.168.1.110";
    double      gripper_open_position   = 0.0;
    double      gripper_close_position  = 100.0;
    double      gripper_pregrasp_position = 40.0;
    double      gripper_close_speed_percent = 50.0;
    double      gripper_close_force_percent = 75.0;
    double      gripper_command_timeout_s   = 1.5;
    bool        gripper_wait_for_target     = false;

    // --- Motion logging (CSV + event log) ---------------------------------
    // When logging_enabled = false, no files are opened and the logging
    // hooks are zero-cost in the active path.
    bool        logging_enabled              = false;
    std::string logging_directory            = "logs";
    std::string logging_experiment_name      = "default_test";
    bool        logging_motion_csv_enabled   = true;
    bool        logging_event_log_enabled    = true;
    int         logging_flush_every_n        = 20;
    bool        logging_only_when_active     = true;
    // If true, the motion CSV samples include the robot's actual TCP pose.
    // The active path NEVER polls getCurrentPose() - the last cached pose
    // (refreshed only while !active) is used. Outside active mode the cache
    // is refreshed every 0.5 s, so this is effectively always available.
    bool        logging_include_actual_pose  = true;

    // --- Runtime config hot-reload ---------------------------------------
    // When runtime_tuning_enabled = false, the config file is not watched
    // and the program behaves exactly as before (single load at start).
    bool runtime_tuning_enabled                 = false;
    bool runtime_tuning_watch_config_file       = true;
    int  runtime_tuning_poll_interval_ms        = 500;
    bool runtime_tuning_print_changes_to_console = true;
    bool runtime_tuning_log_changes_to_event_file = true;
    bool runtime_tuning_apply_only_whitelisted  = true;
    bool runtime_tuning_reject_invalid_values   = true;

    // --- Debug / console summary -----------------------------------------
    // Compact periodic console summary independent of the ConsoleUI render
    // (which clears the screen). Useful for tuning without opening logs.
    bool   debug_print_motion_summary = false;
    double debug_summary_period_s     = 1.0;
    bool   debug_verbose_robot_commands = false;
};

// Loads config from disk. Missing keys keep their defaults. Returns true
// on successful parse (even if some keys were missing).
bool loadConfig(const std::string& path, Config& out);

} // namespace dgd

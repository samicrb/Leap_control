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
    // Bring-up defaults: very low speeds / accels so the joint-level
    // safety supervisor cannot trip on dynamics (alarm 5.7056). Decel
    // ramp + envelope still apply on top.
    int    loop_rate_hz    = 60;
    double max_lin_speed   = 15.0;
    double max_ang_speed   = 2.0;
    double max_lin_accel   = 50.0;
    double max_ang_accel   = 10.0;

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
    // Bring-up defaults: very small. Tune up once decel ramp + alarm
    // budget are confirmed stable.
    double position_scale    = 0.05;
    double orientation_scale = 0.03;
    double position_deadzone_mm = 2.5;
    double orientation_deadzone_deg = 2.0;
    double smoothing_alpha   = 0.25;
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
    // The active control loop now schedules SHORT, NON-BLENDED amovel
    // commands at micro_command_rate_hz instead of streaming speedl()
    // every tick. This removes the failure mode where alarm 5.7056
    // (OPERATION_SAFETY_FUNCTION_SOS_VIOLATION) was tripped by the
    // joint-accel supervisor on a continuous velocity profile.
    double micro_command_rate_hz       = 5.0;    // robot command issue rate
    double micro_min_period_s          = 0.20;   // hard lower bound between commands
    double micro_max_delta_xyz_mm      = 5.0;    // per-command position step cap
    double micro_max_delta_rot_deg     = 1.5;    // per-command orientation step cap
    double micro_deadband_mm           = 0.3;    // skip command if delta below
    double micro_deadband_deg          = 0.2;
    // amovel motion profile (radius=0 always, no blending).
    double micro_lin_vel               = 30.0;   // mm/s
    double micro_ang_vel               = 10.0;   // deg/s
    double micro_lin_acc               = 100.0;  // mm/s^2
    double micro_ang_acc               = 30.0;   // deg/s^2
};

// Loads config from disk. Missing keys keep their defaults. Returns true
// on successful parse (even if some keys were missing).
bool loadConfig(const std::string& path, Config& out);

} // namespace dgd

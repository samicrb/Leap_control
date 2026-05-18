#pragma once

// Strongly-typed config snapshot loaded from config/demo_config.ini.
// Plain key=value format, no external dependencies.

#include <string>
#include <unordered_map>

namespace dgd {

struct Config {
    // --- robot ---
    std::string robot_ip       = "192.168.1.2";
    int         robot_port     = 12345;
    std::string robot_model    = "A0509";
    double      connect_timeout_s = 5.0;

    // --- loop ---
    int    loop_rate_hz    = 60;
    double max_lin_speed   = 120.0;
    double max_ang_speed   = 25.0;
    double max_lin_accel   = 400.0;
    double max_ang_accel   = 120.0;

    // --- workspace envelope (mm, deg) ---
    // When ws_enabled = false the box and orientation cone are NOT enforced
    // (only hard speed/accel caps still apply). Useful for initial bring-up
    // before the cell has been characterised.
    bool   ws_enabled = false;
    double ws_x_min = -300.0, ws_x_max = 400.0;
    double ws_y_min = 200.0,  ws_y_max = 600.0;
    double ws_z_min = 150.0,  ws_z_max = 600.0;
    double ws_rx_range = 45.0, ws_ry_range = 45.0, ws_rz_range = 90.0;

    // --- safe pose (robot BASE frame) ---
    double safe_x  =  55.0, safe_y = 400.0, safe_z = 375.0;
    double safe_rx =  33.0, safe_ry =  96.0, safe_rz = 110.0;

    // --- gesture ---
    double grab_closed_threshold = 0.85;
    double grab_open_threshold   = 0.30;
    double min_confidence        = 0.3;
    double hand_loss_timeout_s   = 0.15;
    double posture_hold_s        = 0.10;

    // --- motion mapping ---
    double position_scale    = 0.8;
    double orientation_scale = 0.6;
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

    // --- gripper backend selection (vendor-agnostic) ---
    // The application picks a gripper implementation via GripperFactory:
    //   enabled = false                         -> NoopGripper
    //   type    = softclaw  + backend = qb_sdk  -> QbSoftClawGripper
    //                                              (qbRobotics SoftClaw
    //                                              connected to THIS PC
    //                                              via USB / RS-485,
    //                                              NOT to the Doosan
    //                                              controller)
    //   backend = tool_io                       -> ToolIoGripperController
    //                                              (legacy DO-driven path)
    //   backend = none                          -> NoopGripper
    // See docs/SOFTCLAW_INTEGRATION.md for the SoftClaw integration
    // notes (SDK URL, COM port discovery, Windows driver hints).
    bool        gripper_enabled              = false;
    std::string gripper_type                 = "softclaw";
    std::string gripper_backend              = "qb_sdk";

    // SoftClaw / qbAPI serial settings. The qbAPI default Windows baud
    // is 2 000 000 - keep this configurable in case a vendor firmware
    // ships with a different rate.
    std::string gripper_port                 = "COM3";
    int         gripper_device_id            = 1;
    int         gripper_baudrate             = 2000000;

    // If false, an initialization failure falls back to NoopGripper
    // (the demo still runs without the gripper). If true, startup aborts.
    bool        gripper_required             = false;
    bool        gripper_initialize_on_startup = true;
    bool        gripper_open_on_startup      = false;
    bool        gripper_stop_on_exit         = true;

    // SoftClaw command values. The qbAPI commSetInputs() expects two
    // shorts: inputs[0] = position, inputs[1] = stiffness / deflection.
    int         gripper_open_position        = 0;
    int         gripper_close_position       = 19000;
    int         gripper_open_deflection      = 0;
    int         gripper_close_deflection     = 15000;

    // Rate-limit / duplicate-suppression on the SoftClaw bus.
    int         gripper_min_command_period_ms = 30;
    int         gripper_command_deadband     = 50;
    int         gripper_command_timeout_ms   = 100;

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
};

// Loads config from disk. Missing keys keep their defaults. Returns true
// on successful parse (even if some keys were missing).
bool loadConfig(const std::string& path, Config& out);

} // namespace dgd

#include "config/Config.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace dgd {

namespace {

std::string trim(const std::string& s) {
    auto a = s.find_first_not_of(" \t\r\n");
    auto b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

bool parseBool(const std::string& v, bool fallback) {
    std::string t = v;
    std::transform(t.begin(), t.end(), t.begin(), ::tolower);
    if (t == "true" || t == "1" || t == "yes" || t == "on")  return true;
    if (t == "false" || t == "0" || t == "no" || t == "off") return false;
    return fallback;
}

double parseDouble(const std::string& v, double fallback) {
    try { return std::stod(v); } catch (...) { return fallback; }
}

int parseInt(const std::string& v, int fallback) {
    try { return std::stoi(v); } catch (...) { return fallback; }
}

} // namespace

namespace {

// Reorganized demo_config.ini uses [section] headers. The canonical keys
// the rest of the code reads (e.g. robot.micro_command_rate_hz) are kept
// for backward compatibility; the table below maps the new section.key
// form back to the canonical flat key. A new-form value wins over an
// old-form one ONLY if the canonical key is not already set explicitly.
struct Alias { const char* from; const char* to; };

const Alias kAliases[] = {
    // [leap]
    {"leap.min_confidence",        "gesture.min_confidence"},
    {"leap.hand_loss_timeout_s",   "gesture.hand_loss_timeout_s"},
    {"leap.posture_hold_s",        "gesture.posture_hold_s"},
    {"leap.grab_closed_threshold", "gesture.grab_closed_threshold"},
    {"leap.grab_open_threshold",   "gesture.grab_open_threshold"},

    // [robot_connection]
    {"robot_connection.ip",                "robot.ip"},
    {"robot_connection.port",              "robot.port"},
    {"robot_connection.model",             "robot.model"},
    {"robot_connection.connect_timeout_s", "robot.connect_timeout_s"},
    {"robot_connection.skip_move_home",    "robot.skip_move_home"},

    // [safety]
    {"safety.collision_sensitivity", "robot.collision_sensitivity"},
    {"safety.singularity_handling",  "robot.singularity_handling"},
    {"safety.home_use_movejx",       "robot.home_use_movejx"},
    {"safety.auto_reset_safety",     "robot.auto_reset_safety"},

    // [motion_mapping]
    {"motion_mapping.position_scale",          "motion.position_scale"},
    {"motion_mapping.orientation_scale",       "motion.orientation_scale"},
    {"motion_mapping.position_deadzone_mm",    "motion.position_deadzone_mm"},
    {"motion_mapping.orientation_deadzone_deg","motion.orientation_deadzone_deg"},
    {"motion_mapping.sign_x",  "motion.sign_x"},
    {"motion_mapping.sign_y",  "motion.sign_y"},
    {"motion_mapping.sign_z",  "motion.sign_z"},
    {"motion_mapping.sign_rx", "motion.sign_rx"},
    {"motion_mapping.sign_ry", "motion.sign_ry"},
    {"motion_mapping.sign_rz", "motion.sign_rz"},

    // [motion_smoothing]
    {"motion_smoothing.smoothing_alpha", "motion.smoothing_alpha"},

    // [micro_pursuit]
    {"micro_pursuit.enabled",             "robot.micro_pursuit_enabled"},
    {"micro_pursuit.hand_to_robot_ratio", "robot.micro_hand_to_robot_ratio"},
    {"micro_pursuit.min_step_xyz_mm",     "robot.micro_min_step_xyz_mm"},
    {"micro_pursuit.max_step_xyz_mm",     "robot.micro_max_step_xyz_mm"},
    {"micro_pursuit.min_step_rot_deg",    "robot.micro_min_step_rot_deg"},
    {"micro_pursuit.max_step_rot_deg",    "robot.micro_max_step_rot_deg"},
    {"micro_pursuit.arrival_band_xyz_mm", "robot.micro_arrival_band_xyz_mm"},
    {"micro_pursuit.arrival_band_rot_deg","robot.micro_arrival_band_rot_deg"},
    {"micro_pursuit.tracking_recovery_time_s","robot.micro_tracking_recovery_time_s"},

    // [robot_command]
    {"robot_command.micro_command_rate_hz","robot.micro_command_rate_hz"},
    {"robot_command.micro_min_period_s",   "robot.micro_min_period_s"},
    {"robot_command.micro_lin_vel",        "robot.micro_lin_vel"},
    {"robot_command.micro_ang_vel",        "robot.micro_ang_vel"},
    {"robot_command.micro_lin_acc",        "robot.micro_lin_acc"},
    {"robot_command.micro_ang_acc",        "robot.micro_ang_acc"},
    {"robot_command.micro_min_step_xyz_mm","robot.micro_min_step_xyz_mm"},
    {"robot_command.micro_max_step_xyz_mm","robot.micro_max_step_xyz_mm"},
    {"robot_command.micro_min_step_rot_deg","robot.micro_min_step_rot_deg"},
    {"robot_command.micro_max_step_rot_deg","robot.micro_max_step_rot_deg"},
    {"robot_command.prevent_command_backlog",  "robot.prevent_command_backlog"},
    {"robot_command.min_motion_completion_ratio","robot.min_motion_completion_ratio"},
    {"robot_command.max_pending_command_age_s","robot.max_pending_command_age_s"},

    // [blending]
    {"blending.enabled",   "robot.micro_blending_enabled"},
    {"blending.radius_mm", "robot.micro_blending_radius_mm"},
    {"blending.type",      "robot.micro_blending_type"},

    // [velocity_filter]
    {"velocity_filter.enabled",        "robot.micro_velocity_filter_enabled"},
    {"velocity_filter.alpha",          "robot.micro_velocity_filter_alpha"},
    {"velocity_filter.max_jerk_xyz",   "robot.micro_max_jerk_xyz"},
    {"velocity_filter.max_jerk_rot",   "robot.micro_max_jerk_rot"},
    {"velocity_filter.deadband_mm_s",  "robot.micro_velocity_deadband_mm_s"},
    {"velocity_filter.stop_ramp_time_s","robot.micro_stop_ramp_time_s"},
};

} // namespace

bool loadConfig(const std::string& path, Config& c) {
    std::ifstream in(path);
    if (!in) {
        LOG_W("Config file '%s' not found - using built-in defaults.", path.c_str());
        return false;
    }

    // Parse the file in two flavours:
    //   - canonical flat keys of the form "namespace.key" win
    //   - "[section] key=value" lines are translated to "section.key";
    //     the alias table above maps a few of those to canonical names
    //     so the existing code paths keep working.
    std::unordered_map<std::string, std::string> kv;
    std::string line;
    std::string current_section;
    while (std::getline(in, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        std::string stripped = trim(line);
        if (stripped.empty()) continue;
        if (stripped.front() == '[' && stripped.back() == ']') {
            current_section = trim(stripped.substr(1, stripped.size() - 2));
            continue;
        }
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = trim(line.substr(0, eq));
        std::string v = trim(line.substr(eq + 1));
        if (k.empty()) continue;
        // Bare key under an active section is stored as "section.key".
        // If the key already contains a dot the user gave a canonical
        // name; keep it as-is so existing INIs keep loading unchanged.
        if (!current_section.empty() && k.find('.') == std::string::npos) {
            k = current_section + "." + k;
        }
        kv[k] = v;
    }

    // Promote new-form (section.key) values to their canonical flat keys
    // when the canonical name was not explicitly provided. Old keys win:
    // this keeps existing demo_config.ini files behaving identically.
    for (const auto& a : kAliases) {
        auto it_from = kv.find(a.from);
        if (it_from == kv.end()) continue;
        if (kv.find(a.to) == kv.end()) {
            kv[a.to] = it_from->second;
        }
    }

    auto sGet = [&](const std::string& k, const std::string& fallback) {
        auto it = kv.find(k); return it == kv.end() ? fallback : it->second;
    };
    auto dGet = [&](const std::string& k, double fallback) {
        auto it = kv.find(k); return it == kv.end() ? fallback : parseDouble(it->second, fallback);
    };
    auto iGet = [&](const std::string& k, int fallback) {
        auto it = kv.find(k); return it == kv.end() ? fallback : parseInt(it->second, fallback);
    };
    auto bGet = [&](const std::string& k, bool fallback) {
        auto it = kv.find(k); return it == kv.end() ? fallback : parseBool(it->second, fallback);
    };

    c.robot_ip           = sGet("robot.ip", c.robot_ip);
    c.robot_port         = iGet("robot.port", c.robot_port);
    c.robot_model        = sGet("robot.model", c.robot_model);
    c.connect_timeout_s  = dGet("robot.connect_timeout_s", c.connect_timeout_s);
    c.skip_move_home     = bGet("robot.skip_move_home", c.skip_move_home);
    c.return_home_on_start    = bGet("robot.return_home_on_start",
                                     c.return_home_on_start);
    c.return_home_on_shutdown = bGet("robot.return_home_on_shutdown",
                                     c.return_home_on_shutdown);
    c.collision_sensitivity = iGet("robot.collision_sensitivity", c.collision_sensitivity);
    c.singularity_handling  = iGet("robot.singularity_handling",  c.singularity_handling);
    c.home_use_movejx       = bGet("robot.home_use_movejx",       c.home_use_movejx);
    c.auto_reset_safety     = bGet("robot.auto_reset_safety",     c.auto_reset_safety);

    c.loop_rate_hz   = iGet("loop.rate_hz", c.loop_rate_hz);
    c.max_lin_speed  = dGet("robot.max_lin_speed", c.max_lin_speed);
    c.max_ang_speed  = dGet("robot.max_ang_speed", c.max_ang_speed);
    c.max_lin_accel  = dGet("robot.max_lin_accel", c.max_lin_accel);
    c.max_ang_accel  = dGet("robot.max_ang_accel", c.max_ang_accel);

    c.ws_enabled = bGet("workspace.enabled", c.ws_enabled);
    c.ws_x_min = dGet("workspace.x_min", c.ws_x_min);
    c.ws_x_max = dGet("workspace.x_max", c.ws_x_max);
    c.ws_y_min = dGet("workspace.y_min", c.ws_y_min);
    c.ws_y_max = dGet("workspace.y_max", c.ws_y_max);
    c.ws_z_min = dGet("workspace.z_min", c.ws_z_min);
    c.ws_z_max = dGet("workspace.z_max", c.ws_z_max);
    c.ws_rx_range = dGet("workspace.rx_range", c.ws_rx_range);
    c.ws_ry_range = dGet("workspace.ry_range", c.ws_ry_range);
    c.ws_rz_range = dGet("workspace.rz_range", c.ws_rz_range);

    c.safe_x  = dGet("safe_pose.x",  c.safe_x);
    c.safe_y  = dGet("safe_pose.y",  c.safe_y);
    c.safe_z  = dGet("safe_pose.z",  c.safe_z);
    c.safe_rx = dGet("safe_pose.rx", c.safe_rx);
    c.safe_ry = dGet("safe_pose.ry", c.safe_ry);
    c.safe_rz = dGet("safe_pose.rz", c.safe_rz);

    c.grab_closed_threshold = dGet("gesture.grab_closed_threshold", c.grab_closed_threshold);
    c.grab_open_threshold   = dGet("gesture.grab_open_threshold",   c.grab_open_threshold);
    c.min_confidence        = dGet("gesture.min_confidence",        c.min_confidence);
    c.hand_loss_timeout_s   = dGet("gesture.hand_loss_timeout_s",   c.hand_loss_timeout_s);
    c.posture_hold_s        = dGet("gesture.posture_hold_s",        c.posture_hold_s);

    c.position_scale    = dGet("motion.position_scale",    c.position_scale);
    c.orientation_scale = dGet("motion.orientation_scale", c.orientation_scale);
    c.position_deadzone_mm = dGet("motion.position_deadzone_mm", c.position_deadzone_mm);
    c.orientation_deadzone_deg = dGet("motion.orientation_deadzone_deg", c.orientation_deadzone_deg);
    c.smoothing_alpha  = dGet("motion.smoothing_alpha",   c.smoothing_alpha);
    c.sign_x  = iGet("motion.sign_x",  c.sign_x);
    c.sign_y  = iGet("motion.sign_y",  c.sign_y);
    c.sign_z  = iGet("motion.sign_z",  c.sign_z);
    c.sign_rx = iGet("motion.sign_rx", c.sign_rx);
    c.sign_ry = iGet("motion.sign_ry", c.sign_ry);
    c.sign_rz = iGet("motion.sign_rz", c.sign_rz);

    c.gripper_open_mm  = dGet("gripper.distance_open_mm",  c.gripper_open_mm);
    c.gripper_close_mm = dGet("gripper.distance_close_mm", c.gripper_close_mm);
    c.gripper_neutral_min_mm = dGet("gripper.neutral_min_mm", c.gripper_neutral_min_mm);
    c.gripper_neutral_max_mm = dGet("gripper.neutral_max_mm", c.gripper_neutral_max_mm);
    c.gripper_facing_dot_max = dGet("gripper.facing_dot_max", c.gripper_facing_dot_max);
    c.gripper_cooldown_s     = dGet("gripper.cooldown_s",     c.gripper_cooldown_s);
    c.gripper_gesture_hold_s = dGet("gripper.gesture_hold_s", c.gripper_gesture_hold_s);
    c.gripper_open_do_index  = iGet("gripper.open_do_index",  c.gripper_open_do_index);
    c.gripper_close_do_index = iGet("gripper.close_do_index", c.gripper_close_do_index);
    c.gripper_pulse_high_ms       = iGet("gripper.pulse_high_ms",       c.gripper_pulse_high_ms);
    c.gripper_test_pulse_on_start = bGet("gripper.test_pulse_on_start", c.gripper_test_pulse_on_start);
    c.gripper_test_pulse_index    = iGet("gripper.test_pulse_index",    c.gripper_test_pulse_index);

    // --- Vendor / optional gripper selection ---
    c.gripper_enabled              = bGet("gripper.enabled",              c.gripper_enabled);
    c.gripper_type                 = sGet("gripper.type",                 c.gripper_type);
    c.gripper_backend              = sGet("gripper.backend",              c.gripper_backend);
    c.gripper_ip                   = sGet("gripper.ip",                   c.gripper_ip);
    c.gripper_open_position        = dGet("gripper.open_position",        c.gripper_open_position);
    c.gripper_close_position       = dGet("gripper.close_position",       c.gripper_close_position);
    c.gripper_pregrasp_position    = dGet("gripper.pregrasp_position",    c.gripper_pregrasp_position);
    c.gripper_close_speed_percent  = dGet("gripper.close_speed_percent",  c.gripper_close_speed_percent);
    c.gripper_close_force_percent  = dGet("gripper.close_force_percent",  c.gripper_close_force_percent);
    c.gripper_command_timeout_s    = dGet("gripper.command_timeout_s",    c.gripper_command_timeout_s);
    c.gripper_wait_for_target      = bGet("gripper.wait_for_target",      c.gripper_wait_for_target);

    // --- Tool (TCP + Tool Weight) ---
    c.tool_apply_on_start   = bGet("tool.apply_on_start",   c.tool_apply_on_start);
    c.tool_tcp_name         = sGet("tool.tcp_name",         c.tool_tcp_name);
    c.tool_tool_weight_name = sGet("tool.tool_weight_name", c.tool_tool_weight_name);

    c.button_mode = sGet("button.mode", c.button_mode);
    c.button_key  = sGet("button.keyboard_key", c.button_key);

    c.ui_clear_each_tick = bGet("ui.clear_screen_each_tick", c.ui_clear_each_tick);
    c.ui_show_hand_debug = bGet("ui.show_hand_debug", c.ui_show_hand_debug);

    c.log_level = sGet("log.level", c.log_level);
    c.log_file  = sGet("log.file",  c.log_file);

    c.dryrun_robot   = bGet("dryrun.robot",   c.dryrun_robot);
    c.dryrun_gripper = bGet("dryrun.gripper", c.dryrun_gripper);

    c.micro_command_rate_hz   = dGet("robot.micro_command_rate_hz",   c.micro_command_rate_hz);
    c.micro_min_period_s      = dGet("robot.micro_min_period_s",      c.micro_min_period_s);
    c.micro_max_delta_xyz_mm  = dGet("robot.micro_max_delta_xyz_mm",  c.micro_max_delta_xyz_mm);
    c.micro_max_delta_rot_deg = dGet("robot.micro_max_delta_rot_deg", c.micro_max_delta_rot_deg);
    c.micro_deadband_mm       = dGet("robot.micro_deadband_mm",       c.micro_deadband_mm);
    c.micro_deadband_deg      = dGet("robot.micro_deadband_deg",      c.micro_deadband_deg);
    c.micro_lin_vel           = dGet("robot.micro_lin_vel",           c.micro_lin_vel);
    c.micro_ang_vel           = dGet("robot.micro_ang_vel",           c.micro_ang_vel);
    c.micro_lin_acc           = dGet("robot.micro_lin_acc",           c.micro_lin_acc);
    c.micro_ang_acc           = dGet("robot.micro_ang_acc",           c.micro_ang_acc);
    c.micro_blending_enabled = bGet("robot.micro_blending_enabled", c.micro_blending_enabled);
    c.micro_blending_radius_mm = dGet("robot.micro_blending_radius_mm", c.micro_blending_radius_mm);
    c.micro_blending_type = sGet("robot.micro_blending_type", c.micro_blending_type);

    c.micro_blending_enabled   = bGet("robot.micro_blending_enabled",   c.micro_blending_enabled);
    c.micro_blending_radius_mm = dGet("robot.micro_blending_radius_mm", c.micro_blending_radius_mm);
    c.micro_blending_type      = sGet("robot.micro_blending_type",      c.micro_blending_type);

    c.micro_pursuit_enabled      = bGet("robot.micro_pursuit_enabled",      c.micro_pursuit_enabled);
    c.micro_hand_to_robot_ratio  = dGet("robot.micro_hand_to_robot_ratio",  c.micro_hand_to_robot_ratio);
    c.micro_min_step_xyz_mm      = dGet("robot.micro_min_step_xyz_mm",      c.micro_min_step_xyz_mm);
    c.micro_max_step_xyz_mm      = dGet("robot.micro_max_step_xyz_mm",      c.micro_max_step_xyz_mm);
    c.micro_min_step_rot_deg     = dGet("robot.micro_min_step_rot_deg",     c.micro_min_step_rot_deg);
    c.micro_max_step_rot_deg     = dGet("robot.micro_max_step_rot_deg",     c.micro_max_step_rot_deg);
    c.micro_arrival_band_xyz_mm  = dGet("robot.micro_arrival_band_xyz_mm",  c.micro_arrival_band_xyz_mm);
    c.micro_arrival_band_rot_deg = dGet("robot.micro_arrival_band_rot_deg", c.micro_arrival_band_rot_deg);

    c.micro_velocity_filter_enabled = bGet("robot.micro_velocity_filter_enabled", c.micro_velocity_filter_enabled);
    c.micro_velocity_filter_alpha   = dGet("robot.micro_velocity_filter_alpha",   c.micro_velocity_filter_alpha);
    c.micro_max_jerk_xyz            = dGet("robot.micro_max_jerk_xyz",            c.micro_max_jerk_xyz);
    c.micro_max_jerk_rot            = dGet("robot.micro_max_jerk_rot",            c.micro_max_jerk_rot);
    c.micro_velocity_deadband_mm_s  = dGet("robot.micro_velocity_deadband_mm_s",  c.micro_velocity_deadband_mm_s);
    c.micro_stop_ramp_time_s        = dGet("robot.micro_stop_ramp_time_s",        c.micro_stop_ramp_time_s);

    c.micro_tracking_recovery_time_s = dGet("robot.micro_tracking_recovery_time_s",
                                            c.micro_tracking_recovery_time_s);
    c.prevent_command_backlog     = bGet("robot.prevent_command_backlog",
                                         c.prevent_command_backlog);
    c.min_motion_completion_ratio = dGet("robot.min_motion_completion_ratio",
                                         c.min_motion_completion_ratio);
    c.max_pending_command_age_s   = dGet("robot.max_pending_command_age_s",
                                         c.max_pending_command_age_s);

    // [tracking_loss_tolerance] - canonical keys use the section.key form
    // directly because the parser already prefixes bare keys under a
    // section with "section." (Config.cpp parser).
    c.tracking_loss_tolerance_enabled = bGet("tracking_loss_tolerance.enabled",
                                              c.tracking_loss_tolerance_enabled);
    c.tracking_loss_frame_timeout_s   = dGet("tracking_loss_tolerance.frame_timeout_s",
                                              c.tracking_loss_frame_timeout_s);
    c.tracking_loss_brief_timeout_s   = dGet("tracking_loss_tolerance.brief_loss_timeout_s",
                                              c.tracking_loss_brief_timeout_s);
    c.tracking_loss_recovery_stability_s = dGet("tracking_loss_tolerance.recovery_stability_s",
                                                  c.tracking_loss_recovery_stability_s);
    c.tracking_loss_freeze_target_on_loss = bGet("tracking_loss_tolerance.freeze_target_on_loss",
                                                  c.tracking_loss_freeze_target_on_loss);
    c.tracking_loss_reanchor_on_recovery  = bGet("tracking_loss_tolerance.reanchor_on_recovery",
                                                  c.tracking_loss_reanchor_on_recovery);
    c.tracking_loss_reset_velocity_on_recovery = bGet("tracking_loss_tolerance.reset_velocity_on_recovery",
                                                       c.tracking_loss_reset_velocity_on_recovery);
    c.tracking_loss_ramp_to_zero_on_loss  = bGet("tracking_loss_tolerance.ramp_to_zero_on_loss",
                                                  c.tracking_loss_ramp_to_zero_on_loss);
    c.tracking_loss_fault_after_timeout   = bGet("tracking_loss_tolerance.fault_after_timeout",
                                                  c.tracking_loss_fault_after_timeout);
    c.tracking_loss_require_recenter_after_brief_loss =
        bGet("tracking_loss_tolerance.require_recenter_after_brief_loss",
             c.tracking_loss_require_recenter_after_brief_loss);
    c.tracking_loss_max_recovery_step_xyz_mm =
        dGet("tracking_loss_tolerance.max_recovery_step_xyz_mm",
             c.tracking_loss_max_recovery_step_xyz_mm);
    c.tracking_loss_max_recovery_step_rot_deg =
        dGet("tracking_loss_tolerance.max_recovery_step_rot_deg",
             c.tracking_loss_max_recovery_step_rot_deg);
    c.tracking_loss_recovery_soft_commands =
        iGet("tracking_loss_tolerance.recovery_soft_commands",
             c.tracking_loss_recovery_soft_commands);

    c.logging_enabled             = bGet("logging.enabled",              c.logging_enabled);
    c.logging_directory           = sGet("logging.log_directory",        c.logging_directory);
    c.logging_experiment_name     = sGet("logging.experiment_name",      c.logging_experiment_name);
    c.logging_motion_csv_enabled  = bGet("logging.motion_csv_enabled",   c.logging_motion_csv_enabled);
    c.logging_event_log_enabled   = bGet("logging.event_log_enabled",    c.logging_event_log_enabled);
    c.logging_flush_every_n       = iGet("logging.flush_every_n_samples",c.logging_flush_every_n);
    c.logging_only_when_active    = bGet("logging.log_only_when_active", c.logging_only_when_active);
    c.logging_include_actual_pose = bGet("logging.include_robot_actual_pose",
                                         c.logging_include_actual_pose);

    c.runtime_tuning_enabled                  = bGet("runtime_tuning.enabled",
                                                     c.runtime_tuning_enabled);
    c.runtime_tuning_watch_config_file        = bGet("runtime_tuning.watch_config_file",
                                                     c.runtime_tuning_watch_config_file);
    c.runtime_tuning_poll_interval_ms         = iGet("runtime_tuning.poll_interval_ms",
                                                     c.runtime_tuning_poll_interval_ms);
    c.runtime_tuning_print_changes_to_console = bGet("runtime_tuning.print_changes_to_console",
                                                     c.runtime_tuning_print_changes_to_console);
    c.runtime_tuning_log_changes_to_event_file= bGet("runtime_tuning.log_changes_to_event_file",
                                                     c.runtime_tuning_log_changes_to_event_file);
    c.runtime_tuning_apply_only_whitelisted   = bGet("runtime_tuning.apply_only_whitelisted_parameters",
                                                     c.runtime_tuning_apply_only_whitelisted);
    c.runtime_tuning_reject_invalid_values    = bGet("runtime_tuning.reject_invalid_values",
                                                     c.runtime_tuning_reject_invalid_values);

    c.debug_print_motion_summary  = bGet("debug.print_motion_summary",  c.debug_print_motion_summary);
    c.debug_summary_period_s      = dGet("debug.summary_period_s",      c.debug_summary_period_s);
    c.debug_verbose_robot_commands= bGet("debug.verbose_robot_commands",c.debug_verbose_robot_commands);

    LOG_I("Config loaded from %s (%zu keys)", path.c_str(), kv.size());
    return true;
}

} // namespace dgd

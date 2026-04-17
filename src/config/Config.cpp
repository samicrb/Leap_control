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

bool loadConfig(const std::string& path, Config& c) {
    std::ifstream in(path);
    if (!in) {
        LOG_W("Config file '%s' not found - using built-in defaults.", path.c_str());
        return false;
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = trim(line.substr(0, eq));
        std::string v = trim(line.substr(eq + 1));
        if (!k.empty()) kv[k] = v;
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

    c.loop_rate_hz   = iGet("loop.rate_hz", c.loop_rate_hz);
    c.max_lin_speed  = dGet("robot.max_lin_speed", c.max_lin_speed);
    c.max_ang_speed  = dGet("robot.max_ang_speed", c.max_ang_speed);
    c.max_lin_accel  = dGet("robot.max_lin_accel", c.max_lin_accel);
    c.max_ang_accel  = dGet("robot.max_ang_accel", c.max_ang_accel);

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

    c.button_mode = sGet("button.mode", c.button_mode);
    c.button_key  = sGet("button.keyboard_key", c.button_key);

    c.ui_clear_each_tick = bGet("ui.clear_screen_each_tick", c.ui_clear_each_tick);
    c.ui_show_hand_debug = bGet("ui.show_hand_debug", c.ui_show_hand_debug);

    c.log_level = sGet("log.level", c.log_level);
    c.log_file  = sGet("log.file",  c.log_file);

    c.dryrun_robot   = bGet("dryrun.robot",   c.dryrun_robot);
    c.dryrun_gripper = bGet("dryrun.gripper", c.dryrun_gripper);

    LOG_I("Config loaded from %s (%zu keys)", path.c_str(), kv.size());
    return true;
}

} // namespace dgd

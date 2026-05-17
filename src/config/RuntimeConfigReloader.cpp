#include "config/RuntimeConfigReloader.hpp"

#include "config/Config.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

namespace dgd {

namespace {

long long mtimeNs(const std::string& path) {
    std::error_code ec;
    auto t = std::filesystem::last_write_time(path, ec);
    if (ec) return 0;
    // C++17-portable: the duration's rep is implementation-defined but
    // arithmetic and comparison are well-defined, which is all we need.
    return static_cast<long long>(t.time_since_epoch().count());
}

// Validate a candidate value. Returns true if the value is acceptable
// for the given canonical key. Unknown / non-tunable keys return false.
bool validate(const char* key, double v) {
    auto inRange = [&](double lo, double hi) { return v >= lo && v <= hi; };
    if (std::strcmp(key, "robot.micro_command_rate_hz") == 0)  return inRange(0.5, 50.0);
    if (std::strcmp(key, "robot.micro_min_period_s") == 0)     return v >= 0.01 && v <= 1.0;
    if (std::strcmp(key, "robot.micro_lin_vel") == 0)          return v > 0.0   && v <= 500.0;
    if (std::strcmp(key, "robot.micro_ang_vel") == 0)          return v > 0.0   && v <= 180.0;
    if (std::strcmp(key, "robot.micro_lin_acc") == 0)          return v > 0.0   && v <= 5000.0;
    if (std::strcmp(key, "robot.micro_ang_acc") == 0)          return v > 0.0   && v <= 1500.0;
    if (std::strcmp(key, "robot.micro_blending_radius_mm") == 0) return v >= 0.0 && v <= 100.0;
    if (std::strcmp(key, "robot.micro_hand_to_robot_ratio") == 0) return v > 0.0 && v <= 2.0;
    if (std::strcmp(key, "robot.micro_min_step_xyz_mm") == 0)  return v >= 0.0 && v <= 200.0;
    if (std::strcmp(key, "robot.micro_max_step_xyz_mm") == 0)  return v >= 0.0 && v <= 200.0;
    if (std::strcmp(key, "robot.micro_min_step_rot_deg") == 0) return v >= 0.0 && v <= 90.0;
    if (std::strcmp(key, "robot.micro_max_step_rot_deg") == 0) return v >= 0.0 && v <= 90.0;
    if (std::strcmp(key, "robot.micro_arrival_band_xyz_mm") == 0) return v >= 0.0 && v <= 50.0;
    if (std::strcmp(key, "robot.micro_arrival_band_rot_deg") == 0) return v >= 0.0 && v <= 30.0;
    if (std::strcmp(key, "motion.position_scale") == 0)        return v > 0.0 && v <= 5.0;
    if (std::strcmp(key, "motion.orientation_scale") == 0)     return v > 0.0 && v <= 5.0;
    if (std::strcmp(key, "motion.position_deadzone_mm") == 0)  return v >= 0.0 && v <= 50.0;
    if (std::strcmp(key, "motion.orientation_deadzone_deg")==0)return v >= 0.0 && v <= 30.0;
    if (std::strcmp(key, "motion.smoothing_alpha") == 0)       return v >= 0.0 && v <= 1.0;
    if (std::strcmp(key, "logging.flush_every_n_samples")==0)  return v >= 1.0 && v <= 10000.0;
    if (std::strcmp(key, "debug.summary_period_s") == 0)       return v > 0.0 && v <= 60.0;
    if (std::strcmp(key, "robot.micro_tracking_recovery_time_s")==0) return v >= 0.0 && v <= 5.0;
    if (std::strcmp(key, "robot.min_motion_completion_ratio")==0) return v >= 0.0 && v <= 1.5;
    if (std::strcmp(key, "robot.max_pending_command_age_s")==0)return v > 0.0 && v <= 5.0;
    return true; // bool-only keys handled separately, defaults pass
}

std::string fmt(const char* key, double oldv, double newv) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Parameter changed: %s %.4f -> %.4f",
                  key, oldv, newv);
    return buf;
}

std::string fmtBool(const char* key, bool oldv, bool newv) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Parameter changed: %s %s -> %s",
                  key, oldv ? "true" : "false", newv ? "true" : "false");
    return buf;
}

std::string fmtStr(const char* key, const std::string& oldv, const std::string& newv) {
    char buf[320];
    std::snprintf(buf, sizeof(buf), "Parameter changed: %s '%s' -> '%s'",
                  key, oldv.c_str(), newv.c_str());
    return buf;
}

std::string fmtReject(const char* key, double v) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Invalid runtime parameter rejected: %s = %.4f",
                  key, v);
    return buf;
}

#define TRY_DOUBLE(field, key)                                    \
    if (live_->field != fresh.field) {                            \
        if (!live_->runtime_tuning_reject_invalid_values ||       \
            validate(key, fresh.field)) {                          \
            if (sink_) sink_(fmt(key, live_->field, fresh.field));\
            live_->field = fresh.field;                           \
            ++changes;                                            \
        } else if (sink_) {                                       \
            sink_(fmtReject(key, fresh.field));                   \
        }                                                         \
    }

#define TRY_BOOL(field, key)                                      \
    if (live_->field != fresh.field) {                            \
        if (sink_) sink_(fmtBool(key, live_->field, fresh.field));\
        live_->field = fresh.field;                               \
        ++changes;                                                \
    }

#define TRY_STRING(field, key)                                    \
    if (live_->field != fresh.field) {                            \
        if (sink_) sink_(fmtStr(key, live_->field, fresh.field)); \
        live_->field = fresh.field;                               \
        ++changes;                                                \
    }

} // namespace

void RuntimeConfigReloader::attach(const std::string& config_path,
                                   Config& live, EventSink sink) {
    config_path_ = config_path;
    live_        = &live;
    sink_        = std::move(sink);
    last_mtime_ns_ = mtimeNs(config_path_);
    last_poll_s_   = 0.0;
}

void RuntimeConfigReloader::detach() {
    live_ = nullptr;
    sink_ = {};
    config_path_.clear();
}

bool RuntimeConfigReloader::poll(double now_s) {
    if (!live_) return false;
    const double interval = std::max(0.05,
        static_cast<double>(live_->runtime_tuning_poll_interval_ms) / 1000.0);
    if (now_s - last_poll_s_ < interval) return false;
    last_poll_s_ = now_s;
    if (!live_->runtime_tuning_watch_config_file) return false;

    long long m = mtimeNs(config_path_);
    if (m == 0 || m == last_mtime_ns_) return false;
    last_mtime_ns_ = m;
    return reloadNow();
}

bool RuntimeConfigReloader::reloadNow() {
    if (!live_) return false;
    if (sink_) sink_("Runtime config reload detected");

    Config fresh = *live_;  // start from current state so unspecified keys keep their value
    if (!loadConfig(config_path_, fresh)) {
        if (sink_) sink_("Reload aborted: could not read config file");
        return false;
    }
    return applyDiff(fresh);
}

bool RuntimeConfigReloader::applyDiff(const Config& fresh) {
    int changes = 0;

    // --- Motion mapping ---
    TRY_DOUBLE(position_scale,             "motion.position_scale");
    TRY_DOUBLE(orientation_scale,          "motion.orientation_scale");
    TRY_DOUBLE(position_deadzone_mm,       "motion.position_deadzone_mm");
    TRY_DOUBLE(orientation_deadzone_deg,   "motion.orientation_deadzone_deg");
    TRY_DOUBLE(smoothing_alpha,            "motion.smoothing_alpha");

    // --- Micro pursuit + robot command ---
    TRY_DOUBLE(micro_command_rate_hz,      "robot.micro_command_rate_hz");
    TRY_DOUBLE(micro_min_period_s,         "robot.micro_min_period_s");
    TRY_DOUBLE(micro_lin_vel,              "robot.micro_lin_vel");
    TRY_DOUBLE(micro_ang_vel,              "robot.micro_ang_vel");
    TRY_DOUBLE(micro_lin_acc,              "robot.micro_lin_acc");
    TRY_DOUBLE(micro_ang_acc,              "robot.micro_ang_acc");
    TRY_BOOL  (micro_pursuit_enabled,      "robot.micro_pursuit_enabled");
    TRY_DOUBLE(micro_hand_to_robot_ratio,  "robot.micro_hand_to_robot_ratio");
    TRY_DOUBLE(micro_min_step_xyz_mm,      "robot.micro_min_step_xyz_mm");
    TRY_DOUBLE(micro_max_step_xyz_mm,      "robot.micro_max_step_xyz_mm");
    TRY_DOUBLE(micro_min_step_rot_deg,     "robot.micro_min_step_rot_deg");
    TRY_DOUBLE(micro_max_step_rot_deg,     "robot.micro_max_step_rot_deg");
    TRY_DOUBLE(micro_arrival_band_xyz_mm,  "robot.micro_arrival_band_xyz_mm");
    TRY_DOUBLE(micro_arrival_band_rot_deg, "robot.micro_arrival_band_rot_deg");
    TRY_DOUBLE(micro_tracking_recovery_time_s, "robot.micro_tracking_recovery_time_s");
    TRY_BOOL  (prevent_command_backlog,    "robot.prevent_command_backlog");
    TRY_DOUBLE(min_motion_completion_ratio,"robot.min_motion_completion_ratio");
    TRY_DOUBLE(max_pending_command_age_s,  "robot.max_pending_command_age_s");

    // Step range sanity (max >= min after edits).
    if (live_->micro_max_step_xyz_mm < live_->micro_min_step_xyz_mm) {
        if (sink_) sink_("Invalid pair: micro_max_step_xyz_mm < min - swapping");
        std::swap(live_->micro_max_step_xyz_mm, live_->micro_min_step_xyz_mm);
    }
    if (live_->micro_max_step_rot_deg < live_->micro_min_step_rot_deg) {
        if (sink_) sink_("Invalid pair: micro_max_step_rot_deg < min - swapping");
        std::swap(live_->micro_max_step_rot_deg, live_->micro_min_step_rot_deg);
    }

    // --- Blending ---
    TRY_BOOL  (micro_blending_enabled,     "robot.micro_blending_enabled");
    TRY_DOUBLE(micro_blending_radius_mm,   "robot.micro_blending_radius_mm");
    TRY_STRING(micro_blending_type,        "robot.micro_blending_type");

    // --- Logging (subset is safe to flip at runtime) ---
    TRY_BOOL(logging_enabled,            "logging.enabled");
    TRY_BOOL(logging_motion_csv_enabled, "logging.motion_csv_enabled");
    TRY_BOOL(logging_event_log_enabled,  "logging.event_log_enabled");
    if (live_->logging_flush_every_n != fresh.logging_flush_every_n) {
        const double cand = static_cast<double>(fresh.logging_flush_every_n);
        if (!live_->runtime_tuning_reject_invalid_values ||
            validate("logging.flush_every_n_samples", cand)) {
            if (sink_) sink_(fmt("logging.flush_every_n_samples",
                                 static_cast<double>(live_->logging_flush_every_n),
                                 cand));
            live_->logging_flush_every_n = fresh.logging_flush_every_n;
            ++changes;
        } else if (sink_) {
            sink_(fmtReject("logging.flush_every_n_samples", cand));
        }
    }
    TRY_BOOL(logging_only_when_active,    "logging.log_only_when_active");
    TRY_BOOL(logging_include_actual_pose, "logging.include_robot_actual_pose");

    // --- Debug summary ---
    TRY_BOOL  (debug_print_motion_summary,  "debug.print_motion_summary");
    TRY_DOUBLE(debug_summary_period_s,      "debug.summary_period_s");
    TRY_BOOL  (debug_verbose_robot_commands,"debug.verbose_robot_commands");

    if (changes == 0 && sink_) sink_("Reload completed: no parameters changed");
    return changes > 0;
}

} // namespace dgd

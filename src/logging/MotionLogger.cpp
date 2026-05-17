#include "logging/MotionLogger.hpp"

#include "config/Config.hpp"
#include "robot/RobotPose.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>

namespace dgd {

namespace {

std::string nowStamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm_buf);
    return buf;
}

// Use wall-clock time so CSV timestamps line up with the event log and
// real-world reference. The numeric timestamp_ms column keeps the
// monotonic value the controller actually sees.
std::string isoStampNow() {
    using clock = std::chrono::system_clock;
    auto tp = clock::now();
    auto secs = clock::to_time_t(tp);
    auto ms_part = std::chrono::duration_cast<std::chrono::milliseconds>(
                       tp.time_since_epoch()) % 1000;
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &secs);
#else
    localtime_r(&secs, &tm_buf);
#endif
    int ms = static_cast<int>(ms_part.count());
    if (ms < 0) ms = 0;
    if (ms > 999) ms = 999;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, ms);
    return buf;
}

const char* kHeader =
    "timestamp_iso,timestamp_ms,dt_ms,"
    "tracking_valid,deadman_active,control_mode,command_sent,command_skip_reason,"
    "hand_raw_x,hand_raw_y,hand_raw_z,hand_raw_rx,hand_raw_ry,hand_raw_rz,"
    "hand_filtered_x,hand_filtered_y,hand_filtered_z,"
    "hand_filtered_rx,hand_filtered_ry,hand_filtered_rz,"
    "hand_delta_x,hand_delta_y,hand_delta_z,"
    "hand_delta_rx,hand_delta_ry,hand_delta_rz,"
    "desired_target_x,desired_target_y,desired_target_z,"
    "desired_target_rx,desired_target_ry,desired_target_rz,"
    "last_commanded_x,last_commanded_y,last_commanded_z,"
    "last_commanded_rx,last_commanded_ry,last_commanded_rz,"
    "commanded_x,commanded_y,commanded_z,"
    "commanded_rx,commanded_ry,commanded_rz,"
    "cached_actual_robot_x,cached_actual_robot_y,cached_actual_robot_z,"
    "cached_actual_robot_rx,cached_actual_robot_ry,cached_actual_robot_rz,"
    "position_error_mm,rotation_error_deg,"
    "commanded_step_xyz_mm,commanded_step_rot_deg,"
    "micro_command_rate_hz,micro_min_period_s,"
    "micro_lin_vel,micro_lin_acc,micro_ang_vel,micro_ang_acc,"
    "micro_blending_enabled,micro_blending_radius_mm,"
    "micro_pursuit_enabled,micro_hand_to_robot_ratio,"
    "motion_position_scale,motion_orientation_scale,motion_smoothing_alpha,"
    // Extended diagnostics
    "command_interval_ms,scheduler_elapsed_ms,previous_motion_estimated_time_ms,"
    "backlog_guard_active,loop_overrun_ms,"
    "cached_actual_pose_age_ms,actual_pose_live,"
    "raw_error_x,raw_error_y,raw_error_z,raw_error_rx,raw_error_ry,raw_error_rz,"
    "desired_velocity_x,desired_velocity_y,desired_velocity_z,"
    "desired_velocity_rx,desired_velocity_ry,desired_velocity_rz,"
    "filtered_velocity_x,filtered_velocity_y,filtered_velocity_z,"
    "filtered_velocity_rx,filtered_velocity_ry,filtered_velocity_rz,"
    "limited_accel_x,limited_accel_y,limited_accel_z,"
    "limited_accel_rx,limited_accel_ry,limited_accel_rz,"
    "raw_step_xyz_mm,limited_step_xyz_mm,raw_step_rot_deg,limited_step_rot_deg,"
    "velocity_deadband_applied,jerk_limit_applied,accel_limit_applied,"
    "step_norm_clipped,tracking_stable_age_ms\n";

void writeVec3(std::FILE* f, const std::optional<std::array<double, 3>>& v) {
    if (v) {
        std::fprintf(f, ",%.4f,%.4f,%.4f", (*v)[0], (*v)[1], (*v)[2]);
    } else {
        std::fputs(",,,", f);
    }
}

void writeVec6(std::FILE* f, const std::optional<std::array<double, 6>>& v) {
    if (v) {
        std::fprintf(f, ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                     (*v)[0], (*v)[1], (*v)[2], (*v)[3], (*v)[4], (*v)[5]);
    } else {
        std::fputs(",,,,,,", f);
    }
}

} // namespace

MotionLogger::~MotionLogger() {
    close();
}

bool MotionLogger::open(const Config& cfg) {
    close();
    if (!cfg.logging_enabled || !cfg.logging_motion_csv_enabled) return false;

    std::error_code ec;
    std::filesystem::create_directories(cfg.logging_directory, ec);
    if (ec) {
        LOG_W("MotionLogger: cannot create directory '%s' (%s) - disabling.",
              cfg.logging_directory.c_str(), ec.message().c_str());
        return false;
    }

    path_ = cfg.logging_directory + "/" + cfg.logging_experiment_name + "_" +
            nowStamp() + "_motion.csv";

#if defined(_WIN32)
    std::FILE* f = nullptr;
    if (fopen_s(&f, path_.c_str(), "w") != 0) f = nullptr;
    file_ = f;
#else
    file_ = std::fopen(path_.c_str(), "w");
#endif
    if (!file_) {
        LOG_W("MotionLogger: cannot open '%s' - disabling.", path_.c_str());
        path_.clear();
        return false;
    }

    std::fputs(kHeader, file_);
    std::fflush(file_);
    flush_every_n_ = std::max(1, cfg.logging_flush_every_n);
    pending_ = 0;
    warned_failure_ = false;
    LOG_I("MotionLogger: opened '%s' (flush every %d samples).",
          path_.c_str(), flush_every_n_);
    return true;
}

void MotionLogger::close() {
    if (file_) {
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
        LOG_I("MotionLogger: closed '%s'.", path_.c_str());
    }
    path_.clear();
    pending_ = 0;
}

void MotionLogger::flush() {
    if (file_) {
        std::fflush(file_);
        pending_ = 0;
    }
}

void MotionLogger::append(const MotionLogSample& s) {
    if (!file_) return;

    const std::string iso = isoStampNow();
    const double ts_ms = s.timestamp_s * 1000.0;

    int n = std::fprintf(
        file_,
        "%s,%.3f,%.3f,%d,%d,%s,%d,%s",
        iso.c_str(), ts_ms, s.dt_ms,
        s.tracking_valid ? 1 : 0, s.deadman_active ? 1 : 0,
        s.control_mode ? s.control_mode : "?",
        s.command_sent ? 1 : 0,
        s.command_skip_reason ? s.command_skip_reason : "");

    if (n < 0) {
        if (!warned_failure_) {
            LOG_W("MotionLogger: write error - disabling further writes.");
            warned_failure_ = true;
        }
        close();
        return;
    }

    // Order MUST match kHeader exactly: raw_pos, raw_rot, filtered_pos, filtered_rot.
    writeVec3(file_, s.hand_raw_pos);
    writeVec3(file_, s.hand_raw_rot);
    writeVec3(file_, s.hand_filtered_pos);
    writeVec3(file_, s.hand_filtered_rot);

    std::fprintf(file_, ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                 s.hand_delta_pos[0], s.hand_delta_pos[1], s.hand_delta_pos[2],
                 s.hand_delta_rot[0], s.hand_delta_rot[1], s.hand_delta_rot[2]);

    writeVec6(file_, s.desired_target);
    writeVec6(file_, s.last_commanded);
    writeVec6(file_, s.commanded);
    writeVec6(file_, s.actual_robot);

    std::fprintf(file_,
                 ",%.4f,%.4f,%.4f,%.4f,"
                 "%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f",
                 s.position_error_mm, s.rotation_error_deg,
                 s.commanded_step_xyz_mm, s.commanded_step_rot_deg,
                 s.micro_command_rate_hz, s.micro_min_period_s,
                 s.micro_lin_vel, s.micro_lin_acc,
                 s.micro_ang_vel, s.micro_ang_acc,
                 s.micro_blending_enabled ? 1 : 0, s.micro_blending_radius_mm,
                 s.micro_pursuit_enabled ? 1 : 0, s.micro_hand_to_robot_ratio,
                 s.motion_position_scale, s.motion_orientation_scale,
                 s.motion_smoothing_alpha);

    // Extended diagnostics block.
    std::fprintf(file_,
                 ",%.3f,%.3f,%.3f,%d,%.3f,%.3f,%d",
                 s.command_interval_ms, s.scheduler_elapsed_ms,
                 s.previous_motion_estimated_time_ms,
                 s.backlog_guard_active ? 1 : 0,
                 s.loop_overrun_ms,
                 s.cached_actual_pose_age_ms,
                 s.actual_pose_live ? 1 : 0);

    writeVec6(file_, s.raw_error);
    writeVec6(file_, s.desired_velocity);
    writeVec6(file_, s.filtered_velocity);
    writeVec6(file_, s.limited_accel);

    std::fprintf(file_,
                 ",%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%.3f\n",
                 s.raw_step_xyz_mm, s.limited_step_xyz_mm,
                 s.raw_step_rot_deg, s.limited_step_rot_deg,
                 s.velocity_deadband_applied ? 1 : 0,
                 s.jerk_limit_applied ? 1 : 0,
                 s.accel_limit_applied ? 1 : 0,
                 s.step_norm_clipped ? 1 : 0,
                 s.tracking_stable_age_ms);

    if (++pending_ >= flush_every_n_) flush();
}

} // namespace dgd

#pragma once

// MotionLogger - structured CSV log of every motion tick.
//
// Writes one row per Application::tick() (or only while active, when
// log_only_when_active = true). The sample structure carries everything
// the control pipeline computes: raw hand reading, smoothed hand, hand
// delta, pursuit desired target, last-commanded target, the new amovel
// target, the cached robot pose, scalar errors, step magnitudes and a
// snapshot of the tunable parameters in effect this tick.
//
// Design constraints (from CLAUDE_HANDOFF):
//   - MUST NOT crash the control loop when the file system is unavailable
//     (disk full, permissions): on first failure, log a warning and
//     silently disable further writes.
//   - MUST NOT block the time-critical path: writes go to a small in-RAM
//     buffer and are flushed every N samples.
//   - When disabled, append() is a no-op.

#include <array>
#include <cstdio>
#include <optional>
#include <string>

namespace dgd {

struct RobotPose;
struct Config;

struct MotionLogSample {
    double timestamp_s   = 0.0;
    double dt_ms         = 0.0;

    bool   tracking_valid = false;
    bool   deadman_active = false;
    const char* control_mode  = "?";
    bool   command_sent      = false;
    const char* command_skip_reason = "";

    // Right hand (raw palm), Leap frame, mm. nullopt when no fresh sample.
    std::optional<std::array<double, 3>> hand_raw_pos;
    // Right hand (smoothed palm), Leap frame, mm. nullopt before priming.
    std::optional<std::array<double, 3>> hand_filtered_pos;
    // Estimated raw orientation (extrinsic XYZ Euler, deg) of the right
    // hand from (palm_direction, palm_normal). Cheap-derived to keep
    // gesture path unchanged.
    std::optional<std::array<double, 3>> hand_raw_rot;
    std::optional<std::array<double, 3>> hand_filtered_rot;

    // Hand delta vs reference (post-deadzone). mm / deg.
    std::array<double, 3> hand_delta_pos {0, 0, 0};
    std::array<double, 3> hand_delta_rot {0, 0, 0};

    // Pursuit pipeline poses (robot BASE frame, mm/deg).
    std::optional<std::array<double, 6>> desired_target;
    std::optional<std::array<double, 6>> last_commanded;
    std::optional<std::array<double, 6>> commanded;
    std::optional<std::array<double, 6>> actual_robot;

    // Scalar diagnostics.
    double position_error_mm  = 0.0;
    double rotation_error_deg = 0.0;
    double commanded_step_xyz_mm = 0.0;
    double commanded_step_rot_deg = 0.0;

    // Snapshot of relevant tuning parameters at this tick.
    double micro_command_rate_hz = 0.0;
    double micro_min_period_s    = 0.0;
    double micro_lin_vel         = 0.0;
    double micro_lin_acc         = 0.0;
    double micro_ang_vel         = 0.0;
    double micro_ang_acc         = 0.0;
    bool   micro_blending_enabled = false;
    double micro_blending_radius_mm = 0.0;
    bool   micro_pursuit_enabled  = false;
    double micro_hand_to_robot_ratio = 0.0;
    double motion_position_scale     = 0.0;
    double motion_orientation_scale  = 0.0;
    double motion_smoothing_alpha    = 0.0;

    // --- Extended diagnostic columns -----------------------------------
    // Timing.
    double command_interval_ms              = 0.0; // gap to previous send
    double scheduler_elapsed_ms             = 0.0; // elapsed since last_command_sent_s_
    double previous_motion_estimated_time_ms = 0.0; // from last step + lin/ang vel
    bool   backlog_guard_active             = false;
    double loop_overrun_ms                  = 0.0; // dt_ms - 1000/loop_rate_hz

    // Cached actual-pose freshness. The active path never refreshes pose
    // (alarm 5.7056 root cause), so this is "age since last !active poll".
    double cached_actual_pose_age_ms = 0.0;
    bool   actual_pose_live          = false;

    // Velocity-filter per-stage trace (XYZ then RXYZ). nullopt unless
    // the velocity-filter path actually ran this tick.
    std::optional<std::array<double, 6>> raw_error;
    std::optional<std::array<double, 6>> desired_velocity;
    std::optional<std::array<double, 6>> filtered_velocity;
    std::optional<std::array<double, 6>> limited_accel;
    // Raw step magnitudes BEFORE the vector-norm cap. The "limited"
    // values equal the final commanded_step_*.
    double raw_step_xyz_mm     = 0.0;
    double limited_step_xyz_mm = 0.0;
    double raw_step_rot_deg    = 0.0;
    double limited_step_rot_deg= 0.0;
    bool   velocity_deadband_applied = false;
    bool   jerk_limit_applied        = false;
    bool   accel_limit_applied       = false;
    // True when the vector-norm cap actively scaled the integrated step.
    bool   step_norm_clipped         = false;

    // Tracking-stability gate.
    double tracking_stable_age_ms = 0.0; // 0 = invalid this tick

    // Brief tracking-loss tolerance diagnostics.
    bool   tracking_loss_tolerance_enabled = false;
    bool   tracking_recent                = false;
    bool   brief_tracking_loss_active     = false;
    bool   hard_tracking_loss             = false;
    double tracking_loss_duration_ms      = 0.0;
    double tracking_recovery_stable_ms    = 0.0;
    bool   tracking_hold_active           = false;
    bool   reanchor_performed             = false;
    bool   recovery_step_limited          = false;

    // --- Motion backend selection (servol / amovel) ---
    const char* motion_backend     = "?";
    bool        amovel_enabled     = false;   // allow_amovel
    bool        servol_enabled     = false;   // allow_servol && servol_enabled
    const char* command_type       = "?";     // "amovel" | "servol" | ""
    // Servol-specific tuning snapshot for the row (so post-mortem
    // analysis can compare same-run servol windows with different
    // hot-tuned params).
    double servol_time_s    = 0.0;
    double servol_lin_vel   = 0.0;
    double servol_ang_vel   = 0.0;
    double servol_lin_acc   = 0.0;
    double servol_ang_acc   = 0.0;
};

class MotionLogger {
public:
    MotionLogger() = default;
    ~MotionLogger();

    MotionLogger(const MotionLogger&) = delete;
    MotionLogger& operator=(const MotionLogger&) = delete;

    // Opens the CSV file under <directory>/<experiment>_<YYYY-MM-DD_HH-MM-SS>_motion.csv.
    // Writes the header on success. Returns true if logging is ready.
    bool open(const Config& cfg);
    void close();

    // Record one sample. No-op when not open / disabled.
    void append(const MotionLogSample& sample);

    // Force a flush regardless of buffer size.
    void flush();

    bool isOpen() const { return file_ != nullptr; }
    const std::string& path() const { return path_; }

private:
    std::FILE*  file_     = nullptr;
    std::string path_;
    int         flush_every_n_ = 20;
    int         pending_       = 0;
    bool        warned_failure_ = false;
};

} // namespace dgd

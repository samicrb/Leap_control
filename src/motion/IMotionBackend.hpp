#pragma once

// IMotionBackend - abstracts the per-tick command emission path.
//
// Two implementations live alongside each other on this branch:
//   AmovelMotionBackend  - existing discrete-segment pursuit with blending
//   ServolMotionBackend  - continuous SERVO-L task-space target update
//
// The Application owns the shared upstream pipeline (tracking gate,
// state machine, hold/recovery, gesture mapping, CSV logging). The
// backend owns whatever internal state is specific to its motion
// primitive and decides whether/what to send to the robot each tick.
//
// Hot-switching the backend mid-run is unsafe (a servol stream colliding
// with an in-flight amovel will reliably trip alarm 5.7056). The
// Application enforces this by deferring any motion_backend.type change
// until the SM returns to a passive state.

#include "robot/RobotPose.hpp"

#include <array>
#include <optional>
#include <string>

namespace dgd {

class IRobotController;
struct Config;

enum class MotionBackendKind {
    Amovel,
    Servol,
};

const char* motionBackendName(MotionBackendKind k);
MotionBackendKind parseMotionBackendKind(const std::string& s, bool* ok = nullptr);

// One per-tick result. All step magnitudes are Euclidean vector norms,
// never per-axis maxima. Pose snapshots are stored as 6-element arrays
// so the result matches the CSV writer's expected types directly.
struct MotionTickResult {
    bool        command_sent = false;
    const char* skip_reason  = "";
    // The pose the backend actually sent (servol target / amovel target).
    std::optional<std::array<double, 6>> commanded_target;
    // Internal target snapshot the backend used as the basis for this tick.
    std::optional<std::array<double, 6>> desired_target;
    std::optional<std::array<double, 6>> last_commanded;
    // Step magnitudes for the CSV log (mm / deg).
    double raw_step_xyz_mm  = 0.0;
    double raw_step_rot_deg = 0.0;
    double commanded_step_xyz_mm  = 0.0;
    double commanded_step_rot_deg = 0.0;
    bool   step_norm_clipped = false;
    // Elapsed time since the previous command this backend sent (ms).
    double command_interval_ms = 0.0;
};

// Read-only view of the Application's per-tick context. Backends never
// mutate the Application's state directly; they only react to it.
struct MotionTickContext {
    double      now_s = 0.0;
    double      tick_elapsed_s = 0.0; // dt
    bool        deadman_active  = false;
    bool        cur_active      = false;   // SM is in POSITION/ORIENTATION
    bool        prev_active     = false;
    bool        tracking_recent = false;   // within frame_timeout_s
    bool        tracking_stable = false;   // sustained for recovery_time_s
    bool        brief_loss_active = false;
    bool        tracking_hold_active = false;
    bool        ramp_to_zero_active = false;
    // SM-issued velocity request (mm/s, deg/s). For the servol path,
    // this is interpreted as a hand-displacement * scale - we recover
    // hand displacement by dividing back by position_scale / orient_scale.
    std::array<double, 3> sm_linear_velocity  {0, 0, 0};
    std::array<double, 3> sm_angular_velocity {0, 0, 0};
    // Reference pose captured on active-mode entry. Backends should
    // anchor their target relative to this.
    RobotPose   active_entry_pose;
    bool        active_entry_valid = false;
    // Recovery soft-cap counter; backends should use it to clip their
    // step on the first N commands after a re-anchor.
    int         recovery_soft_commands_remaining = 0;
};

class IMotionBackend {
public:
    virtual ~IMotionBackend() = default;

    virtual const char* name() const = 0;
    virtual MotionBackendKind kind() const = 0;

    // Called once when the Application is constructed. Stash any
    // references the backend needs; do NOT touch the robot here.
    virtual void attach(const Config& cfg, IRobotController& robot) = 0;

    // Called on the rising edge cur_active=true. seed_pose is the live
    // robot pose at entry (from a single allowed getCurrentPose call).
    // Backends use it to anchor their internal target.
    virtual void onActiveEntry(const RobotPose& seed_pose, double now_s) = 0;

    // Called on the falling edge cur_active=false. Backend should stop
    // emitting and clear its internal target.
    virtual void onActiveExit(double now_s) = 0;

    // Called when tracking has recovered after a brief loss. Backend
    // should pin its target to last_commanded (avoid jumps).
    virtual void onReanchor(const RobotPose& anchor_pose, double now_s) = 0;

    // Per-tick entry point. Backend decides whether/what to send.
    virtual MotionTickResult onTick(const MotionTickContext& ctx) = 0;

    // Returns the current internal target (servol target / amovel
    // last-commanded). Used for logging only.
    virtual std::optional<RobotPose> currentTarget() const = 0;
};

} // namespace dgd

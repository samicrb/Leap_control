#include "motion/ServolMotionBackend.hpp"

#include "config/Config.hpp"
#include "robot/IRobotController.hpp"
#include "util/Logger.hpp"
#include "util/MathUtils.hpp"

#include <algorithm>
#include <cmath>

namespace dgd {

void ServolMotionBackend::attach(const Config& cfg, IRobotController& robot) {
    cfg_   = &cfg;
    robot_ = &robot;
}

void ServolMotionBackend::onActiveEntry(const RobotPose& seed_pose, double now_s) {
    target_       = seed_pose;
    target_valid_ = true;
    // Backdate last_command_s_ so the first tick can fire immediately
    // (otherwise the operator would see a one-period dead spot).
    last_command_s_ = now_s - (cfg_ ? cfg_->servol_min_period_s : 0.05);
    if (!logged_init_) {
        LOG_I("SERVOL backend initialized "
              "(rate=%.1f Hz, min_period=%.3fs, vel=[%.1f mm/s, %.1f deg/s], "
              "acc=[%.0f mm/s^2, %.0f deg/s^2], time=%.3fs, "
              "max_step=[%.1f mm, %.2f deg], arrival_band=[%.1f mm, %.2f deg]).",
              cfg_->servol_command_rate_hz,
              cfg_->servol_min_period_s,
              cfg_->servol_lin_vel, cfg_->servol_ang_vel,
              cfg_->servol_lin_acc, cfg_->servol_ang_acc,
              cfg_->servol_time_s,
              cfg_->servol_max_step_xyz_mm, cfg_->servol_max_step_rot_deg,
              cfg_->servol_arrival_band_xyz_mm, cfg_->servol_arrival_band_rot_deg);
        logged_init_ = true;
    }
}

void ServolMotionBackend::onActiveExit(double /*now_s*/) {
    target_valid_   = false;
    last_command_s_ = -1.0;
}

void ServolMotionBackend::onReanchor(const RobotPose& anchor_pose, double /*now_s*/) {
    target_ = anchor_pose;
}

MotionTickResult ServolMotionBackend::onTick(const MotionTickContext& ctx) {
    MotionTickResult r;
    if (!cfg_ || !robot_) {
        r.skip_reason = "backend_unattached";
        return r;
    }
    if (!target_valid_ || !ctx.active_entry_valid) {
        r.skip_reason = "not_ready";
        return r;
    }

    // Tracking gate. The backend honours both the global tracking-loss
    // tolerance state (already classified by Application) and its own
    // stop_on_tracking_loss / hold_last_target_on_tracking_loss config.
    const bool hold_target =
        ctx.tracking_hold_active || ctx.brief_loss_active || !ctx.tracking_recent;
    if (hold_target && cfg_->servol_stop_on_tracking_loss &&
        !cfg_->servol_hold_last_target_on_tracking_loss) {
        // Honour stop-on-loss: skip emission entirely.
        r.skip_reason = "tracking_invalid";
        return r;
    }

    // Recover hand displacement from cmd.linear_velocity. Application
    // populates ctx.sm_linear_velocity = position_scale * hand_delta;
    // we divide back by the scale and re-apply the pursuit ratio so
    // servol receives the same hand-to-robot mapping as amovel.
    const double inv_pos = (cfg_->position_scale    > 1e-6)
                         ? 1.0 / cfg_->position_scale    : 0.0;
    const double inv_rot = (cfg_->orientation_scale > 1e-6)
                         ? 1.0 / cfg_->orientation_scale : 0.0;
    const double ratio   = cfg_->micro_hand_to_robot_ratio;

    RobotPose desired = ctx.active_entry_pose;
    if (!hold_target) {
        desired.x  += ratio * ctx.sm_linear_velocity[0]  * inv_pos;
        desired.y  += ratio * ctx.sm_linear_velocity[1]  * inv_pos;
        desired.z  += ratio * ctx.sm_linear_velocity[2]  * inv_pos;
        desired.rx += ratio * ctx.sm_angular_velocity[0] * inv_rot;
        desired.ry += ratio * ctx.sm_angular_velocity[1] * inv_rot;
        desired.rz += ratio * ctx.sm_angular_velocity[2] * inv_rot;
    } else {
        // hold_last_target_on_tracking_loss path: keep the current
        // internal target. desired = target_ so delta is zero.
        desired = target_;
    }

    r.desired_target = desired.toArray();
    r.last_commanded = target_.toArray();

    // Compute the raw delta (desired - current internal target) and
    // its Euclidean norms BEFORE any capping. Note: target_ is NOT
    // advanced yet - servol semantics are "keep sending the latest
    // bounded target"; an aborted tick (arrival band / scheduler) must
    // NOT secretly move the target.
    double dx = desired.x  - target_.x;
    double dy = desired.y  - target_.y;
    double dz = desired.z  - target_.z;
    double drx = desired.rx - target_.rx;
    double dry = desired.ry - target_.ry;
    double drz = desired.rz - target_.rz;
    r.raw_step_xyz_mm  = vec3Norm(dx, dy, dz);
    r.raw_step_rot_deg = vec3Norm(drx, dry, drz);

    // Vector-norm cap on the per-command step. Soft-cap on top of it
    // during the post-recovery window (first N commands after a brief-
    // loss re-anchor).
    const bool in_soft = ctx.recovery_soft_commands_remaining > 0;
    const double cap_xyz = in_soft
        ? std::min(cfg_->tracking_loss_max_recovery_step_xyz_mm,
                   cfg_->servol_max_step_xyz_mm)
        : cfg_->servol_max_step_xyz_mm;
    const double cap_rot = in_soft
        ? std::min(cfg_->tracking_loss_max_recovery_step_rot_deg,
                   cfg_->servol_max_step_rot_deg)
        : cfg_->servol_max_step_rot_deg;
    const bool clip_xyz = limitVectorNorm3(dx, dy, dz, cap_xyz);
    const bool clip_rot = limitVectorNorm3(drx, dry, drz, cap_rot);
    r.step_norm_clipped = clip_xyz || clip_rot;

    const double step_xyz = vec3Norm(dx, dy, dz);
    const double step_rot = vec3Norm(drx, dry, drz);
    r.commanded_step_xyz_mm  = step_xyz;
    r.commanded_step_rot_deg = step_rot;

    // Scheduler. servol is meant to be called at a STABLE rate; we do
    // not chase the loop rate (which can have jitter from the UI / log).
    const double elapsed = (last_command_s_ < 0.0)
                         ? 1e9
                         : (ctx.now_s - last_command_s_);
    const double period_floor =
        std::max(cfg_->servol_min_period_s,
                 1.0 / std::max(cfg_->servol_command_rate_hz, 0.1));
    if (elapsed < period_floor) {
        r.skip_reason = "scheduler";
        return r;
    }

    // Arrival band: below this threshold we don't bother sending a
    // refresh - servol keeps the previous target alive on the controller.
    if (step_xyz < cfg_->servol_arrival_band_xyz_mm &&
        step_rot < cfg_->servol_arrival_band_rot_deg) {
        r.skip_reason = "arrival_band";
        return r;
    }

    // Commit the (capped) delta into target_. This happens ONLY when we
    // are actually emitting; an aborted tick leaves target_ unchanged so
    // the next emit sees the original delta.
    target_.x  += dx;
    target_.y  += dy;
    target_.z  += dz;
    target_.rx += drx;
    target_.ry += dry;
    target_.rz += drz;

    if (cfg_->servol_log_diagnostics) {
        LOG_D("servol: target=[%.2f %.2f %.2f / %.2f %.2f %.2f] "
              "step=%.2f mm, %.2f deg (raw %.2f / %.2f) elapsed=%.3fs",
              target_.x, target_.y, target_.z,
              target_.rx, target_.ry, target_.rz,
              step_xyz, step_rot,
              r.raw_step_xyz_mm, r.raw_step_rot_deg, elapsed);
    }

    if (!robot_->sendCartesianServoL(target_,
                                     cfg_->servol_lin_vel,
                                     cfg_->servol_ang_vel,
                                     cfg_->servol_lin_acc,
                                     cfg_->servol_ang_acc,
                                     cfg_->servol_time_s)) {
        r.skip_reason = "robot_refused";
        LOG_W("SERVOL command refused: %s", robot_->lastError().c_str());
        return r;
    }

    r.command_interval_ms = (last_command_s_ < 0.0)
                          ? 0.0
                          : (ctx.now_s - last_command_s_) * 1000.0;
    last_command_s_       = ctx.now_s;
    r.command_sent        = true;
    r.commanded_target    = target_.toArray();
    return r;
}

} // namespace dgd

// Unit test: ServolMotionBackend - target update, vector-norm cap,
// arrival band, scheduler, hold on tracking loss.

#include "config/Config.hpp"
#include "motion/ServolMotionBackend.hpp"
#include "robot/IRobotController.hpp"
#include "robot/RobotPose.hpp"
#include "util/Logger.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace dgd;

namespace {

// Recording stub robot: every sendCartesianServoL call is appended to
// `commands` so the test can assert on the sequence.
class RecordingRobot : public IRobotController {
public:
    struct Cmd {
        RobotPose pose;
        double lin_vel, ang_vel, lin_acc, ang_acc, time_s;
    };
    std::vector<Cmd> commands;

    bool connect(const std::string&, int, double) override { return true; }
    void disconnect() override {}
    bool isConnected() const override { return true; }
    bool engage() override { return true; }
    void disengage() override {}
    bool moveHome(const RobotPose&) override { return true; }
    bool sendCartesianVelocity(const std::array<double, 6>&) override { return true; }
    void stopMotion() override {}
    void emergencyStop() override {}
    bool sendCartesianMicroMove(const RobotPose&, double, double, double, double,
                                double) override { return true; }
    bool sendCartesianServoL(const RobotPose& p, double lv, double av,
                             double la, double aa, double t) override {
        commands.push_back({p, lv, av, la, aa, t});
        return true;
    }
    bool getCurrentPose(RobotPose&) override { return true; }
    std::string lastError() const override { return ""; }
};

void expectClose(double a, double b, double tol, const char* label) {
    if (std::fabs(a - b) > tol) {
        std::fprintf(stderr, "FAIL %s: expected %.6f, got %.6f (tol=%.6f)\n",
                     label, b, a, tol);
        std::exit(1);
    }
    std::printf("  ok: %s = %.4f\n", label, a);
}

double norm3(double a, double b, double c) {
    return std::sqrt(a * a + b * b + c * c);
}

MotionTickContext makeCtx(double now_s, const RobotPose& entry,
                          double cmd_x_mm = 0.0, double cmd_y_mm = 0.0,
                          double cmd_z_mm = 0.0) {
    MotionTickContext ctx;
    ctx.now_s              = now_s;
    ctx.deadman_active     = true;
    ctx.cur_active         = true;
    ctx.tracking_recent    = true;
    ctx.tracking_stable    = true;
    ctx.active_entry_pose  = entry;
    ctx.active_entry_valid = true;
    // sm_linear_velocity is "position_scale * hand_delta" per servol
    // backend recovery math. With position_scale = 1.0 and ratio = 1.0
    // the backend will reproduce these exact mm offsets in the desired
    // target relative to active_entry_pose.
    ctx.sm_linear_velocity = { cmd_x_mm, cmd_y_mm, cmd_z_mm };
    return ctx;
}

} // namespace

int main() {
    Logger::instance().configure(LogLevel::Error, ""); // keep test output quiet

    RecordingRobot robot;
    Config cfg;
    cfg.position_scale            = 1.0;
    cfg.orientation_scale         = 1.0;
    cfg.micro_hand_to_robot_ratio = 1.0;
    cfg.servol_command_rate_hz    = 20.0;
    cfg.servol_min_period_s       = 0.05;
    cfg.servol_lin_vel            = 80.0;
    cfg.servol_ang_vel            = 20.0;
    cfg.servol_lin_acc            = 300.0;
    cfg.servol_ang_acc            = 80.0;
    cfg.servol_time_s             = 0.08;
    cfg.servol_max_step_xyz_mm    = 6.0;
    cfg.servol_max_step_rot_deg   = 1.5;
    cfg.servol_arrival_band_xyz_mm  = 1.0;
    cfg.servol_arrival_band_rot_deg = 0.3;
    cfg.servol_stop_on_tracking_loss             = true;
    cfg.servol_hold_last_target_on_tracking_loss = true;
    cfg.servol_log_diagnostics                   = false;

    ServolMotionBackend backend;
    backend.attach(cfg, robot);

    RobotPose entry{100.0, 200.0, 300.0, 10.0, 20.0, 30.0};
    backend.onActiveEntry(entry, /*now_s=*/0.0);

    // 1. Small hand offset within arrival band -> no command.
    {
        // 0.5 mm offset on X = below 1.0 mm arrival band.
        auto ctx = makeCtx(0.10, entry, 0.5, 0.0, 0.0);
        auto r = backend.onTick(ctx);
        if (r.command_sent) {
            std::fprintf(stderr, "FAIL: command sent below arrival band\n");
            return 1;
        }
        std::printf("  ok: arrival band suppresses small offset (reason=%s)\n",
                    r.skip_reason);
    }

    // 2. Large diagonal hand offset (22, 22, 22 mm) MUST scale to a
    //    vector norm of 6 mm exactly (servol_max_step_xyz_mm).
    {
        auto ctx = makeCtx(0.20, entry, 22.0, 22.0, 22.0);
        auto r = backend.onTick(ctx);
        if (!r.command_sent) {
            std::fprintf(stderr, "FAIL: expected command, got skip=%s\n", r.skip_reason);
            return 1;
        }
        // The recorded command's pose - entry should have a vector norm
        // of exactly 6 mm (within numerical tolerance).
        const auto& sent = robot.commands.back().pose;
        const double dn = norm3(sent.x - entry.x,
                                sent.y - entry.y,
                                sent.z - entry.z);
        expectClose(dn, 6.0, 1e-6,
                    "diagonal hand step caps to vector norm 6 mm (not sqrt(3)*6)");
        expectClose(r.commanded_step_xyz_mm, 6.0, 1e-6,
                    "result reports norm 6 mm");
        // raw step was sqrt(3)*22 ~ 38.1 mm.
        expectClose(r.raw_step_xyz_mm, std::sqrt(3.0) * 22.0, 1e-6,
                    "raw step records the original ~38.1 mm");
        if (!r.step_norm_clipped) {
            std::fprintf(stderr, "FAIL: step_norm_clipped expected true\n");
            return 1;
        }
        // Vel/acc arguments forwarded to DRFL exactly as configured.
        expectClose(robot.commands.back().lin_vel, 80.0, 1e-6, "lin_vel forwarded");
        expectClose(robot.commands.back().ang_vel, 20.0, 1e-6, "ang_vel forwarded");
        expectClose(robot.commands.back().lin_acc, 300.0, 1e-6, "lin_acc forwarded");
        expectClose(robot.commands.back().ang_acc, 80.0, 1e-6, "ang_acc forwarded");
        expectClose(robot.commands.back().time_s, 0.08, 1e-6, "time_s forwarded");
    }

    // 3. Scheduler skip: a tick too soon after the previous emit must
    //    skip with reason "scheduler", regardless of step size.
    {
        const size_t before = robot.commands.size();
        // The previous tick was at t=0.20. min_period_s = 0.05. So a tick
        // at t=0.22 (20 ms later) must skip.
        auto ctx = makeCtx(0.22, entry, 22.0, 22.0, 22.0);
        auto r = backend.onTick(ctx);
        if (r.command_sent || robot.commands.size() != before) {
            std::fprintf(stderr, "FAIL: scheduler did not suppress early tick\n");
            return 1;
        }
        std::printf("  ok: scheduler suppresses tick within min_period_s "
                    "(reason=%s)\n", r.skip_reason);
    }

    // 4. After enough time, the next tick should emit again. The step
    //    is bounded by max_step_xyz_mm, so we approach the desired
    //    target gradually.
    {
        const size_t before = robot.commands.size();
        // Walk a few ticks; each one capped to 6 mm.
        for (int i = 0; i < 5; ++i) {
            auto ctx = makeCtx(0.30 + 0.06 * i, entry, 22.0, 22.0, 22.0);
            backend.onTick(ctx);
        }
        const size_t emitted = robot.commands.size() - before;
        if (emitted < 3) {
            std::fprintf(stderr, "FAIL: expected >= 3 commands over 5 ticks, "
                                 "got %zu\n", emitted);
            return 1;
        }
        std::printf("  ok: scheduler permits subsequent ticks (emitted=%zu)\n",
                    emitted);
        // Each emitted step still <= max_step_xyz_mm.
        for (size_t i = before; i < robot.commands.size(); ++i) {
            // The CSV-visible step is the i-vs-(i-1) pose delta.
            const auto& a = (i == 0) ? entry : robot.commands[i - 1].pose;
            const auto& b = robot.commands[i].pose;
            const double dn = norm3(b.x - a.x, b.y - a.y, b.z - a.z);
            if (dn > 6.0 + 1e-6) {
                std::fprintf(stderr, "FAIL: command %zu step norm %.4f > 6\n",
                             i, dn);
                return 1;
            }
        }
        std::printf("  ok: every emitted step <= max_step_xyz_mm = 6.0 mm\n");
    }

    // 5. Tracking loss with hold_last_target=true: the backend keeps
    //    emitting the SAME target pose at scheduler rate.
    {
        // Move clock far enough forward that the scheduler permits emit.
        const size_t before = robot.commands.size();
        const RobotPose held_target = robot.commands.back().pose;
        auto ctx = makeCtx(1.0, entry, 99.0, 99.0, 99.0);
        ctx.tracking_recent      = false;
        ctx.tracking_stable      = false;
        ctx.tracking_hold_active = true;
        ctx.brief_loss_active    = true;
        auto r = backend.onTick(ctx);
        if (!r.command_sent && robot.commands.size() == before) {
            // It is acceptable if servol_hold_last_target_on_tracking_loss
            // dropped to arrival band; but since the step is exactly 0 here
            // (desired == target_), arrival_band path kicks in.
            std::printf("  ok: tracking loss + hold + zero step -> arrival "
                        "band (reason=%s)\n", r.skip_reason);
        } else {
            // Sent a command - it must equal the held target.
            const auto& sent = robot.commands.back().pose;
            expectClose(sent.x, held_target.x, 1e-6, "hold: x unchanged");
            expectClose(sent.y, held_target.y, 1e-6, "hold: y unchanged");
            expectClose(sent.z, held_target.z, 1e-6, "hold: z unchanged");
        }
    }

    // 6. Tracking loss with stop_on_loss=true and hold=false: skip.
    {
        cfg.servol_stop_on_tracking_loss             = true;
        cfg.servol_hold_last_target_on_tracking_loss = false;
        const size_t before = robot.commands.size();
        auto ctx = makeCtx(2.0, entry, 99.0, 99.0, 99.0);
        ctx.tracking_recent      = false;
        ctx.tracking_hold_active = true;
        ctx.brief_loss_active    = true;
        auto r = backend.onTick(ctx);
        if (r.command_sent || robot.commands.size() != before) {
            std::fprintf(stderr, "FAIL: stop_on_loss did not block emit\n");
            return 1;
        }
        std::printf("  ok: stop_on_tracking_loss blocks emit (reason=%s)\n",
                    r.skip_reason);
    }

    // 7. onActiveExit clears internal target.
    {
        backend.onActiveExit(3.0);
        const size_t before = robot.commands.size();
        auto ctx = makeCtx(3.5, entry, 22.0, 22.0, 22.0);
        // After exit, target_valid_ = false -> backend reports not_ready.
        auto r = backend.onTick(ctx);
        if (r.command_sent || robot.commands.size() != before) {
            std::fprintf(stderr, "FAIL: backend emitted after onActiveExit\n");
            return 1;
        }
        if (std::string(r.skip_reason) != "not_ready") {
            std::fprintf(stderr, "FAIL: expected not_ready, got %s\n",
                         r.skip_reason);
            return 1;
        }
        std::printf("  ok: onActiveExit -> backend reports not_ready\n");
    }

    std::printf("All servol backend tests passed.\n");
    return 0;
}

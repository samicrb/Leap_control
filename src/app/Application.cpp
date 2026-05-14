#include "app/Application.hpp"
#include "input/KeyboardButton.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <thread>

namespace dgd {
namespace {
double nowSeconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

bool isActiveState(DemoState s) {
    return s == DemoState::PositionControl;
}
} // namespace

Application::Application(const Config& cfg,
                         ILeapSource& sensor,
                         IRobotController& robot,
                         IGripperController& gripper,
                         IExternalButton& button)
    : cfg_(cfg), sensor_(sensor), robot_(robot), gripper_(gripper), button_(button),
      interpreter_(cfg), sm_(cfg),
      guard_(cfg, {cfg.safe_x, cfg.safe_y, cfg.safe_z, cfg.safe_rx, cfg.safe_ry, cfg.safe_rz}),
      ui_(cfg) {}

bool Application::initialise() {
    if (!sensor_.start()) LOG_W("Leap source start failed - running degraded.");
    if (!button_.start()) return false;
    if (!robot_.connect(cfg_.robot_ip, cfg_.robot_port, cfg_.connect_timeout_s)) return false;
    if (!robot_.engage()) return false;
    return true;
}

void Application::stop() { running_.store(false); }

int Application::run() {
    running_.store(true);
    const auto period = std::chrono::duration<double>(loopPeriod());

    auto* keyboard = dynamic_cast<KeyboardButton*>(&button_);
    while (running_.load()) {
        const auto start = std::chrono::steady_clock::now();
        tick(nowSeconds());
        if (keyboard && keyboard->shutdownRequested()) running_.store(false);
        const auto remain = period - (std::chrono::steady_clock::now() - start);
        if (remain.count() > 0.0) std::this_thread::sleep_for(remain);
    }

    robot_.disconnect();
    sensor_.stop();
    button_.stop();
    return 0;
}

void Application::tick(double now_s) {
    HandFrame frame{};
    const bool fresh = sensor_.pollLatest(frame);
    if (!fresh) frame = last_frame_;
    last_frame_ = frame;

    GestureReport report = interpreter_.update(frame, now_s);
    report.freshFrame = fresh;

    const bool btn = button_.isActive();
    CommandOutput cmd = sm_.step(report, btn, now_s);
    if (cmd.captureReference) interpreter_.captureReference();

    const DemoState cur_state = sm_.state();
    const bool cur_active = isActiveState(cur_state);
    const bool prev_active = isActiveState(prev_state_);

    if (decel_active_ && cur_active) {
        decel_active_ = false;
        decel_step_ = 0;
    }

    if (decel_active_) {
        static constexpr double kDecelFactors[] = {0.75, 0.55, 0.40, 0.25, 0.12, 0.0};
        const int steps = static_cast<int>(sizeof(kDecelFactors) / sizeof(kDecelFactors[0]));
        const int idx = decel_step_ < steps ? decel_step_ : steps - 1;
        std::array<double, 6> twist = {
            decel_start_twist_[0] * kDecelFactors[idx], decel_start_twist_[1] * kDecelFactors[idx], decel_start_twist_[2] * kDecelFactors[idx],
            0.0, 0.0, 0.0
        };
        guard_.clampSpeed(twist);
        robot_.sendCartesianVelocity(twist);
        ++decel_step_;
        if (decel_step_ >= steps) {
            decel_active_ = false;
            decel_step_ = 0;
            decel_start_twist_ = {0,0,0,0,0,0};
            last_active_twist_ = {0,0,0,0,0,0};
        }
    } else if (cur_active) {
        std::array<double, 6> twist = {
            cmd.linear_velocity[0], cmd.linear_velocity[1], cmd.linear_velocity[2],
            0.0, 0.0, 0.0
        };
        guard_.clampSpeed(twist);
        if (twist[0] != 0.0 || twist[1] != 0.0 || twist[2] != 0.0) last_active_twist_ = twist;
        robot_.sendCartesianVelocity(twist);
    } else if (prev_active) {
        decel_active_ = true;
        decel_step_ = 0;
        decel_start_twist_ = last_active_twist_;
    }

    prev_state_ = cur_state;

    ConsoleUI::Frame uiframe;
    uiframe.state = sm_.state();
    uiframe.fault = sm_.faultReason();
    uiframe.button_active = btn;
    uiframe.sensor_ok = report.sensor_ok;
    uiframe.mode_text = cmd.ui_mode_text;
    uiframe.status_text = cmd.ui_status_text;
    uiframe.prompt_text = cmd.ui_prompt_text;
    uiframe.gesture = report;
    uiframe.pose = last_pose_for_ui_;
    uiframe.gripper = gripper_.lastCommandedState();
    uiframe.workspace_limit = false;
    ui_.render(uiframe);
}

} // namespace dgd

#include "gripper/ToolIoGripperController.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <thread>

namespace dgd {

ToolIoGripperController::ToolIoGripperController(const Config& cfg, IRobotController& robot)
    : cfg_(cfg), robot_(robot) {}

bool ToolIoGripperController::connect() {
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper DRY-RUN: connect skipped.");
        connected_.store(true);
        return true;
    }
    // Pneumatic / electric grippers used as end-effectors on Doosan setups
    // are typically driven by tool I/O; the controller takes care of the
    // power switching. So "connect" just verifies the robot link is up.
    connected_.store(robot_.isConnected());
    if (!connected_.load()) {
        last_error_ = "Robot link not connected";
        LOG_E("Gripper: %s", last_error_.c_str());
        return false;
    }
    LOG_I("Gripper connected via robot tool I/O (open DO=%d, close DO=%d).",
          cfg_.gripper_open_do_index, cfg_.gripper_close_do_index);
    return true;
}

void ToolIoGripperController::disconnect() {
    connected_.store(false);
}

// Minimum HIGH duration of the pulse. The SoftHand and most pneumatic
// end-effectors wired on the Doosan tool flange latch on the rising
// edge. The duration is configurable via gripper.pulse_high_ms, clamped
// here to a 50 ms floor so a misconfiguration cannot drop below the
// latching threshold of the SoftHand input stage.
static constexpr int kMinPulseHighMs = 50;

bool ToolIoGripperController::pulseDO(int index) {
    int hold_ms = cfg_.gripper_pulse_high_ms;
    if (hold_ms < kMinPulseHighMs) hold_ms = kMinPulseHighMs;

    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper TOOL_IO DRY-RUN: pulse O%02d for %d ms (skipped - "
              "dryrun.gripper=true)", index, hold_ms);
        return true;
    }
    // Real pulse: assert the tool DO HIGH, wait hold_ms, drop it LOW.
    // If either DRFL call fails we propagate the failure up so the
    // gripper state stays Unknown and the operator sees a clear error.
    if (!robot_.setToolDigitalOutput(index, true)) {
        last_error_ = "Tool DO ON failed: " + robot_.lastError();
        LOG_E("Gripper TOOL_IO: pulse O%02d HIGH FAILED (%s)",
              index, last_error_.c_str());
        return false;
    }
    LOG_I("Gripper TOOL_IO: pulse O%02d HIGH for %d ms", index, hold_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    if (!robot_.setToolDigitalOutput(index, false)) {
        last_error_ = "Tool DO OFF failed: " + robot_.lastError();
        LOG_E("Gripper TOOL_IO: pulse O%02d LOW FAILED (%s)",
              index, last_error_.c_str());
        return false;
    }
    LOG_I("Gripper TOOL_IO: pulse O%02d LOW", index);
    return true;
}

bool ToolIoGripperController::open() {
    if (!connected_.load()) { last_error_ = "not connected"; return false; }
    if (!pulseDO(cfg_.gripper_open_do_index)) return false;
    last_state_ = State::Open;
    LOG_I("Gripper OPEN command issued.");
    return true;
}

bool ToolIoGripperController::close() {
    if (!connected_.load()) { last_error_ = "not connected"; return false; }
    if (!pulseDO(cfg_.gripper_close_do_index)) return false;
    last_state_ = State::Closed;
    LOG_I("Gripper CLOSE command issued.");
    return true;
}

} // namespace dgd

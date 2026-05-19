#include "gripper/ToolIoGripperController.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>

namespace dgd {

namespace {
// gripper.io_scope is parsed as a string. Recognised values are
// "controller" (default - SoftHand wired to the controller box DOs)
// and "tool" (flange DOs). Anything else logs a warning and defaults
// to "controller" so a typo cannot silently route to the wrong bank.
bool useToolScope(const std::string& s) {
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return t == "tool";
}
const char* scopeTag(const std::string& s) {
    return useToolScope(s) ? "TOOL_IO" : "CONTROLLER_IO";
}
} // namespace

ToolIoGripperController::ToolIoGripperController(const Config& cfg, IRobotController& robot)
    : cfg_(cfg), robot_(robot) {}

bool ToolIoGripperController::connect() {
    const bool tool_scope = useToolScope(cfg_.gripper_io_scope);
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper %s DRY-RUN: connect skipped.",
              tool_scope ? "TOOL_IO" : "CONTROLLER_IO");
        connected_.store(true);
        return true;
    }
    // Two-state grippers driven by Doosan digital outputs - either the
    // flange tool DOs (io_scope = tool) or the controller-box DOs
    // (io_scope = controller). In both cases the controller takes care
    // of the power switching, so "connect" just verifies the link.
    connected_.store(robot_.isConnected());
    if (!connected_.load()) {
        last_error_ = "Robot link not connected";
        LOG_E("Gripper: %s", last_error_.c_str());
        return false;
    }
    if (tool_scope) {
        LOG_I("Gripper connected via robot tool I/O "
              "(open DO=%d, close DO=%d).",
              cfg_.gripper_open_do_index, cfg_.gripper_close_do_index);
    } else {
        LOG_I("Gripper connected via robot controller I/O "
              "(open DO=%d, close DO=%d).",
              cfg_.gripper_open_do_index, cfg_.gripper_close_do_index);
    }
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

    const bool tool_scope = useToolScope(cfg_.gripper_io_scope);
    const char* tag       = scopeTag(cfg_.gripper_io_scope);

    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper %s DRY-RUN: pulse O%02d for %d ms (skipped - "
              "dryrun.gripper=true)", tag, index, hold_ms);
        return true;
    }
    // Real pulse: assert the DO HIGH, wait hold_ms, drop it LOW. The
    // routing is selected by [gripper].io_scope:
    //   "controller" -> robot_.setControllerDigitalOutput()  (GPIO_CTRLBOX_*)
    //   "tool"       -> robot_.setToolDigitalOutput()        (GPIO_TOOL_*)
    // If either DRFL call fails we propagate the failure up so the
    // gripper state stays Unknown and the operator sees a clear error.
    auto setDO = [&](int idx, bool value) {
        return tool_scope
            ? robot_.setToolDigitalOutput(idx, value)
            : robot_.setControllerDigitalOutput(idx, value);
    };

    if (!setDO(index, true)) {
        last_error_ = std::string(tag) + " DO ON failed: " + robot_.lastError();
        LOG_E("Gripper %s: pulse O%02d HIGH FAILED (%s)",
              tag, index, last_error_.c_str());
        return false;
    }
    LOG_I("Gripper %s: pulse O%02d HIGH for %d ms", tag, index, hold_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    if (!setDO(index, false)) {
        last_error_ = std::string(tag) + " DO OFF failed: " + robot_.lastError();
        LOG_E("Gripper %s: pulse O%02d LOW FAILED (%s)",
              tag, index, last_error_.c_str());
        return false;
    }
    LOG_I("Gripper %s: pulse O%02d LOW", tag, index);
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

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

bool ToolIoGripperController::pulseDO(int index) {
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper DRY-RUN: pulse DO[%d]", index);
        return true;
    }
    // DRFL-INTEGRATION: call the real tool-DO function here via the
    // robot controller. We intentionally do NOT add a method to
    // IRobotController for this - the gripper is wired to the robot's
    // tool flange, so this is the only place that should know about it.
    // If you prefer to route it through the interface, add:
    //     virtual bool setToolDigitalOutput(int, bool) = 0;
    // and implement it in DrflRobotController.
    //
    // For now we log loudly so it is obvious the wiring step is needed.
    LOG_W("Gripper: TODO connect tool DO[%d] -> end-effector valve.", index);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

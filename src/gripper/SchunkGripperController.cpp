#include "gripper/SchunkGripperController.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <thread>

namespace dgd {

SchunkGripperController::SchunkGripperController(const Config& cfg, IRobotController& robot)
    : cfg_(cfg), robot_(robot) {}

bool SchunkGripperController::connect() {
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper DRY-RUN: connect skipped.");
        connected_.store(true);
        return true;
    }
    // Schunk grippers used as pneumatic end-effectors on Doosan setups
    // are commonly driven by tool I/O; the controller takes care of the
    // power switching. So "connect" just verifies the robot link is up.
    connected_.store(robot_.isConnected());
    if (!connected_.load()) {
        last_error_ = "Robot link not connected";
        LOG_E("Gripper: %s", last_error_.c_str());
        return false;
    }
    LOG_I("Gripper connected via robot tool I/O.");
    return true;
}

void SchunkGripperController::disconnect() {
    connected_.store(false);
}

bool SchunkGripperController::pulseDO(int index) {
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper DRY-RUN: pulse DO[%d]", index);
        return true;
    }
    // DRFL-INTEGRATION: call the real tool-DO function here via the
    // robot controller. We intentionally do NOT add a method to
    // IRobotController for this - the Schunk gripper is wired to the
    // robot's tool flange, so this is the only place that should know
    // about it. If you prefer to route it through the interface, add:
    //     virtual bool setToolDigitalOutput(int, bool) = 0;
    // and implement it in DrflRobotController.
    //
    // For now we log loudly so it is obvious the wiring step is needed.
    LOG_W("Gripper: TODO connect tool DO[%d] -> Schunk valve.", index);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

bool SchunkGripperController::open() {
    if (!connected_.load()) { last_error_ = "not connected"; return false; }
    if (!pulseDO(kOpenDO)) return false;
    last_state_ = State::Open;
    LOG_I("Gripper OPEN command issued.");
    return true;
}

bool SchunkGripperController::close() {
    if (!connected_.load()) { last_error_ = "not connected"; return false; }
    if (!pulseDO(kCloseDO)) return false;
    last_state_ = State::Closed;
    LOG_I("Gripper CLOSE command issued.");
    return true;
}

} // namespace dgd

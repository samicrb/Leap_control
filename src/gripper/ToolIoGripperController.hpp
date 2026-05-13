#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"
#include "robot/IRobotController.hpp"

#include <atomic>
#include <string>

namespace dgd {

// Generic two-state gripper driven via the Doosan controller's tool digital
// I/O. Suitable for any pneumatic / electric end-effector that exposes a
// pair of "open" / "close" coils on the tool flange (Schunk, Robotiq,
// OnRobot, custom). DO channel indices come from config
// (gripper.open_do_index, gripper.close_do_index).
//
// DRFL exposes set_tool_digital_output(int index, bool). The real channel
// indices and polarities must match the integration on the cell.
class ToolIoGripperController final : public IGripperController {
public:
    ToolIoGripperController(const Config& cfg, IRobotController& robot);

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override { return connected_.load(); }

    bool open() override;
    bool close() override;

    State       lastCommandedState() const override { return last_state_; }
    std::string lastError()          const override { return last_error_; }

private:
    const Config&     cfg_;
    IRobotController& robot_;
    std::atomic<bool> connected_{false};
    State             last_state_ = State::Unknown;
    std::string       last_error_;

    bool pulseDO(int index);
};

} // namespace dgd

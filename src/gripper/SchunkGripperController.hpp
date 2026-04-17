#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"
#include "robot/IRobotController.hpp"

#include <atomic>
#include <string>

namespace dgd {

// Schunk gripper driven via the Doosan controller's tool digital I/O.
//
// Typical wiring for demo setups:
//   - Tool DO #1 -> "close" coil
//   - Tool DO #2 -> "open" coil
//
// DRFL exposes the function set_tool_digital_output(int index, bool).
// The real channel indices and polarities must match your integration.
// Only edit this file + config.
class SchunkGripperController final : public IGripperController {
public:
    SchunkGripperController(const Config& cfg, IRobotController& robot);

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

    // Channel indices used for the demo. Expose via config if needed.
    static constexpr int kCloseDO = 1;
    static constexpr int kOpenDO  = 2;

    bool pulseDO(int index);
};

} // namespace dgd

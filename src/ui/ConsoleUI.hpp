#pragma once

#include "config/Config.hpp"
#include "gesture/GestureTypes.hpp"
#include "gripper/IGripperController.hpp"
#include "robot/RobotPose.hpp"
#include "state/States.hpp"

#include <string>

namespace dgd {

// Minimal console UI. Not the place for pretty graphics - this is a
// supervisor view for the operator. Print an operator-readable block
// on every tick, optionally clearing the screen first. Line count stays
// stable so `clear + print` doesn't flicker.
class ConsoleUI {
public:
    explicit ConsoleUI(const Config& cfg);

    struct Frame {
        DemoState     state = DemoState::Idle;
        FaultReason   fault = FaultReason::None;
        bool          button_active = false;
        bool          sensor_ok = false;
        std::string   mode_text;
        std::string   status_text;
        std::string   prompt_text;
        GestureReport gesture{};
        RobotPose     pose{};
        IGripperController::State gripper = IGripperController::State::Unknown;
        bool          workspace_limit = false;
    };

    void render(const Frame& f);

private:
    const Config& cfg_;
    bool first_render_ = true;
};

} // namespace dgd

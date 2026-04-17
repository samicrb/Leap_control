#pragma once

#include "config/Config.hpp"
#include "gesture/GestureTypes.hpp"
#include "state/States.hpp"
#include "util/MathUtils.hpp"

#include <string>

namespace dgd {

// Actions that Application executes after a state machine tick.
struct CommandOutput {
    // Cartesian velocity request for the robot in base frame
    // (mm/s, deg/s). Zero vector = stop.
    Vec3 linear_velocity  {0.0, 0.0, 0.0};
    Vec3 angular_velocity {0.0, 0.0, 0.0};

    // Hard-stop flag: bypasses any smoothing / velocity streaming and
    // issues an immediate motion halt on the robot controller.
    bool hardStop = false;

    // Gripper impulses forwarded from the gesture layer (one-shot).
    bool openGripper  = false;
    bool closeGripper = false;

    // Reference capture request for the gesture interpreter. Application
    // must call GestureInterpreter::captureReference() when true.
    bool captureReference = false;

    // UI feedback.
    std::string ui_mode_text;
    std::string ui_status_text;
    std::string ui_prompt_text;
};

// StateMachine is a pure function of (previous state, GestureReport,
// external button). It owns no robot/gripper state directly.
class StateMachine {
public:
    explicit StateMachine(const Config& cfg);

    // Run one tick.
    CommandOutput step(const GestureReport& g, bool button_active, double now_s);

    DemoState   state()       const { return state_; }
    FaultReason faultReason() const { return fault_reason_; }

    void reset();

private:
    // Guard helpers.
    bool conditionsForPositionControl(const GestureReport& g) const;
    bool conditionsForOrientationControl(const GestureReport& g) const;
    bool conditionsForRecenter(const GestureReport& g) const;
    FaultReason evaluateFaults(const GestureReport& g, bool button_active) const;

    // Transitions.
    void enter(DemoState s, const GestureReport& g, double now_s,
               CommandOutput& out);

    const Config& cfg_;
    DemoState     state_        = DemoState::Idle;
    FaultReason   fault_reason_ = FaultReason::None;
    double        state_entered_s_ = 0.0;
};

} // namespace dgd

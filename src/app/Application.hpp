#pragma once

#include "config/Config.hpp"
#include "gesture/GestureInterpreter.hpp"
#include "gripper/IGripperController.hpp"
#include "input/IExternalButton.hpp"
#include "robot/IRobotController.hpp"
#include "robot/WorkspaceGuard.hpp"
#include "sensor/ILeapSource.hpp"
#include "state/StateMachine.hpp"
#include "ui/ConsoleUI.hpp"

#include <atomic>
#include <memory>

namespace dgd {

// Application owns the demo lifecycle. Dependencies are injected so the
// whole thing can be driven by mocks in a test binary.
class Application {
public:
    Application(const Config& cfg,
                ILeapSource&       sensor,
                IRobotController&  robot,
                IGripperController& gripper,
                IExternalButton&   button);

    // Prepares everything for the first tick. Moves the robot home.
    bool initialise();

    // Blocking run loop. Exits when stop() is called or the button
    // source signals shutdown.
    int run();

    // Thread-safe shutdown request.
    void stop();

private:
    void tick(double now_s);
    double loopPeriod() const { return 1.0 / static_cast<double>(cfg_.loop_rate_hz); }

    const Config&      cfg_;
    ILeapSource&       sensor_;
    IRobotController&  robot_;
    IGripperController& gripper_;
    IExternalButton&   button_;

    GestureInterpreter interpreter_;
    StateMachine       sm_;
    WorkspaceGuard     guard_;
    ConsoleUI          ui_;

    HandFrame          last_frame_{};
    std::atomic<bool>  running_{false};
    // Latches whether we already fired the one-shot emergencyStop() for
    // the current Fault entry. Reset whenever we leave the Fault state.
    bool               fault_emergency_issued_ = false;
};

} // namespace dgd

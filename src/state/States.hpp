#pragma once

#include <string>

namespace dgd {

// Explicit, finite state machine. No composite / nested states.
enum class DemoState {
    Idle,               // Button not active - strictly no motion.
    Ready,              // Button active, waiting for engagement.
    PositionControl,    // Driving TCP translation.
    OrientationControl, // Driving TCP orientation.
    Recenter,           // Hold while user repositions the right hand.
    Gripper,            // Gripper gesture recognised; motion suspended.
    Fault               // Invalid condition; must return to Ready.
};

const char* stateName(DemoState s);

// Reasons a transition occurred / we entered Fault. Shown briefly on UI
// and written to the log to aid supervision.
enum class FaultReason {
    None,
    SensorDisconnected,
    LeftHandLost,
    RightHandLost,
    RightHandPostureInvalid,
    WorkspaceLimit,
    RobotError,
    InternalError
};

const char* faultReasonText(FaultReason r);

} // namespace dgd

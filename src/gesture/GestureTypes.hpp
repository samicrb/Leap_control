#pragma once

#include "util/MathUtils.hpp"

#include <string>

namespace dgd {

enum class HandPosture { Unknown, Open, Closed };

// Output of GestureInterpreter::update() for one tick.
struct GestureReport {
    double timestamp_s = 0.0;
    bool   sensor_ok = false;

    // High-level hand presence / posture.
    bool leftPresent  = false;
    bool rightPresent = false;
    HandPosture leftPosture  = HandPosture::Unknown;
    HandPosture rightPosture = HandPosture::Unknown;
    double leftConfidence  = 0.0;
    double rightConfidence = 0.0;

    // True when we have a reliable new reading this tick (as opposed to
    // stale data held over because tracking briefly stuttered).
    bool freshFrame = false;

    // Relative right-hand motion since reference capture.
    // Position in mm, orientation delta in deg (extrinsic XYZ Euler on
    // palm direction / normal). These are ZERO until the state machine
    // requests captureReference().
    Vec3 rightDeltaPosition {0.0, 0.0, 0.0};
    Vec3 rightDeltaOrientation {0.0, 0.0, 0.0};

    // --- Gripper gesture evaluation -------------------------------------
    // True when both hands are visible AND facing each other enough to be
    // interpreted as a clap-like two-hand gripper gesture.
    bool gripperGestureArmed = false;
    // Distance between palms in mm. Valid only when both hands present.
    double handDistance_mm = -1.0;

    // Impulse events (latched for one tick). The gesture interpreter sets
    // these only when a threshold crossing occurs AND cooldown allows it.
    bool gripperOpenImpulse  = false;
    bool gripperCloseImpulse = false;

    // Short human-readable tag for UI/log diagnostics.
    std::string tag;
};

} // namespace dgd

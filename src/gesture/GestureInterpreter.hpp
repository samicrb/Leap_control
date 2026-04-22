#pragma once

#include "config/Config.hpp"
#include "gesture/GestureTypes.hpp"
#include "sensor/HandFrame.hpp"

namespace dgd {

// Converts raw HandFrame samples into a GestureReport.
// - Tracks hysteresis for the open/closed posture so the hand isn't
//   flickering between states near the threshold.
// - Maintains a motion reference for relative position / orientation.
// - Detects the two-hand gripper gesture and emits single-tick impulses.
//
// Fully deterministic per input stream: no hidden timers beyond simple
// last-seen timestamps.
class GestureInterpreter {
public:
    explicit GestureInterpreter(const Config& cfg);

    // Feed one frame. Returns a report for this tick.
    GestureReport update(const HandFrame& frame, double now_s);

    // Called by the state machine the moment a control state is entered,
    // so relative deltas start from 0 (prevents jumps on re-engagement).
    void captureReference();

    // Forget any internal smoothing / references. Used on fault recovery.
    void reset();

    // Inspected by the state machine to know whether a reference is
    // currently set.
    bool hasReference() const { return reference_set_; }

private:
    const Config& cfg_;

    HandPosture posture(double grab, HandPosture prev) const;

    // Last-known fresh samples and their timestamps (for stale detection).
    double last_left_s_  = -1.0;
    double last_right_s_ = -1.0;

    HandPosture last_left_posture_  = HandPosture::Unknown;
    HandPosture last_right_posture_ = HandPosture::Unknown;

    // Motion reference.
    bool reference_set_ = false;
    Vec3 ref_right_position_ {0.0, 0.0, 0.0};
    Vec3 ref_right_normal_   {0.0, -1.0, 0.0};
    Vec3 ref_right_direction_{0.0, 0.0, -1.0};

    // Smoothed right-hand sample (keeps motion feeling stable).
    Vec3 smoothed_position_  {0.0, 0.0, 0.0};
    Vec3 smoothed_normal_    {0.0, -1.0, 0.0};
    Vec3 smoothed_direction_ {0.0, 0.0, -1.0};
    bool smoother_primed_ = false;

    // Gripper gesture state.
    enum class GripperZone { Open, Neutral, Close, Unknown };
    GripperZone gripper_last_zone_ = GripperZone::Unknown;
    double      gripper_last_command_s_ = -1000.0;
    double      gripper_armed_since_s_  = -1.0;
};

} // namespace dgd

#include "state/StateMachine.hpp"
#include "util/Logger.hpp"

namespace dgd {

const char* stateName(DemoState s) {
    switch (s) {
        case DemoState::Idle:               return "IDLE";
        case DemoState::Ready:              return "READY";
        case DemoState::PositionControl:    return "POSITION";
        case DemoState::OrientationControl: return "ORIENTATION";
        case DemoState::Recenter:           return "RECENTER";
        case DemoState::Gripper:            return "GRIPPER";
        case DemoState::Fault:              return "FAULT";
    }
    return "?";
}

const char* faultReasonText(FaultReason r) {
    switch (r) {
        case FaultReason::None:                    return "none";
        case FaultReason::SensorDisconnected:      return "sensor disconnected";
        case FaultReason::LeftHandLost:            return "left hand lost";
        case FaultReason::RightHandLost:           return "right hand lost";
        case FaultReason::RightHandPostureInvalid: return "right hand posture invalid";
        case FaultReason::WorkspaceLimit:          return "workspace limit";
        case FaultReason::RobotError:              return "robot error";
        case FaultReason::InternalError:           return "internal error";
    }
    return "?";
}

StateMachine::StateMachine(const Config& cfg) : cfg_(cfg) {}

void StateMachine::reset() {
    state_ = DemoState::Idle;
    fault_reason_ = FaultReason::None;
    state_entered_s_ = 0.0;
}

bool StateMachine::conditionsForPositionControl(const GestureReport& g) const {
    return g.sensor_ok &&
           g.leftPresent  && g.leftPosture  == HandPosture::Open &&
           g.rightPresent && g.rightPosture == HandPosture::Closed &&
           !g.gripperGestureArmed;
}

bool StateMachine::conditionsForOrientationControl(const GestureReport& g) const {
    return g.sensor_ok &&
           g.leftPresent  && g.leftPosture  == HandPosture::Closed &&
           g.rightPresent && g.rightPosture == HandPosture::Closed &&
           !g.gripperGestureArmed;
}

bool StateMachine::conditionsForRecenter(const GestureReport& g) const {
    // Left hand must be valid; right hand visible but not closed.
    return g.sensor_ok &&
           g.leftPresent && g.leftPosture != HandPosture::Unknown &&
           g.rightPresent && g.rightPosture != HandPosture::Closed &&
           !g.gripperGestureArmed;
}

FaultReason StateMachine::evaluateFaults(const GestureReport& g, bool button_active) const {
    if (!button_active) return FaultReason::None; // caller handles -> Idle
    if (!g.sensor_ok)   return FaultReason::SensorDisconnected;
    // Faults are only meaningful once we've left Ready. When the
    // Application has the brief-loss tolerance armed, hand-loss faults
    // are masked so a transient Leap dropout does not boot the SM into
    // FAULT (which would require the operator to lift and re-show both
    // hands). The sensor-disconnect path is always raised.
    if (state_ == DemoState::PositionControl ||
        state_ == DemoState::OrientationControl) {
        if (!suppress_hand_loss_fault_) {
            if (!g.leftPresent)  return FaultReason::LeftHandLost;
            if (!g.rightPresent) return FaultReason::RightHandLost;
        }
    }
    return FaultReason::None;
}

void StateMachine::enter(DemoState s, const GestureReport& /*g*/, double now_s,
                         CommandOutput& out) {
    if (state_ == s) return;
    LOG_I("State: %s -> %s", stateName(state_), stateName(s));
    state_ = s;
    state_entered_s_ = now_s;

    switch (s) {
        case DemoState::PositionControl:
        case DemoState::OrientationControl:
            // Capture a new reference so deltas start at zero.
            out.captureReference = true;
            break;
        case DemoState::Fault:
        case DemoState::Recenter:
        case DemoState::Ready:
        case DemoState::Idle:
        case DemoState::Gripper:
            out.hardStop = true;
            break;
    }
}

CommandOutput StateMachine::step(const GestureReport& g, bool button_active, double now_s) {
    CommandOutput out;
    out.ui_prompt_text = "";

    // Rule 1: no motion without external authorisation.
    if (!button_active) {
        if (state_ != DemoState::Idle) {
            enter(DemoState::Idle, g, now_s, out);
        }
        out.hardStop = true;
        out.ui_mode_text   = "DEMO INACTIVE";
        out.ui_status_text = "Press the start button to enable.";
        out.ui_prompt_text = "Waiting for operator button.";
        fault_reason_ = FaultReason::None;
        return out;
    }

    // Button active but sensor not ready => Fault.
    if (!g.sensor_ok) {
        if (state_ != DemoState::Fault) {
            fault_reason_ = FaultReason::SensorDisconnected;
            enter(DemoState::Fault, g, now_s, out);
        }
        out.hardStop = true;
        out.ui_mode_text   = "HOLD";
        out.ui_status_text = "Tracking lost - place the sensor in view.";
        out.ui_prompt_text = "System will recover automatically.";
        return out;
    }

    // From Idle -> Ready as soon as the button goes active.
    if (state_ == DemoState::Idle) {
        enter(DemoState::Ready, g, now_s, out);
    }

    // Faults: if we're in an active state and something is lost, go Fault.
    FaultReason fr = evaluateFaults(g, button_active);
    if (fr != FaultReason::None && state_ != DemoState::Fault) {
        fault_reason_ = fr;
        enter(DemoState::Fault, g, now_s, out);
    }

    // Run the switch up to two times so a transition triggered inside one
    // case (e.g. Position -> Gripper) immediately gets its own handler
    // applied within the same tick. This avoids "swallowed" impulses.
    for (int pass = 0; pass < 2; ++pass) {
        DemoState before = state_;
        switch (state_) {
        case DemoState::Ready: {
            out.hardStop = true;
            out.ui_mode_text = "READY";
            out.ui_status_text = "Present both hands to engage.";
            if (!g.leftPresent && !g.rightPresent) {
                out.ui_prompt_text = "Show both hands over the sensor.";
            } else if (!g.leftPresent) {
                out.ui_prompt_text = "Show your LEFT hand (mode).";
            } else if (!g.rightPresent) {
                out.ui_prompt_text = "Show your RIGHT hand (control).";
            } else {
                out.ui_prompt_text =
                    "Left OPEN = move | Left CLOSED = turn | then close RIGHT fist.";
            }

            // Gripper gesture has priority (Rule 7) even from Ready.
            if (g.gripperGestureArmed) {
                enter(DemoState::Gripper, g, now_s, out);
                break;
            }

            if (conditionsForPositionControl(g)) {
                enter(DemoState::PositionControl, g, now_s, out);
            } else if (conditionsForOrientationControl(g)) {
                enter(DemoState::OrientationControl, g, now_s, out);
            }
            break;
        }
        case DemoState::PositionControl: {
            out.ui_mode_text = "POSITION MODE";
            out.ui_status_text = "Right-hand fist drives X/Y/Z.";
            out.ui_prompt_text = "Open right hand to pause and recenter.";

            if (g.gripperGestureArmed) {
                enter(DemoState::Gripper, g, now_s, out);
                break;
            }
            if (!conditionsForPositionControl(g)) {
                // Figure out the gentle transition first (recenter vs Fault).
                if (conditionsForRecenter(g)) {
                    enter(DemoState::Recenter, g, now_s, out);
                } else if (conditionsForOrientationControl(g)) {
                    // User flipped left hand to closed: switch mode.
                    enter(DemoState::OrientationControl, g, now_s, out);
                } else {
                    out.hardStop = true;
                }
                break;
            }

            // Produce velocity = scaled delta per control loop period.
            // Use a first-order "proportional" response: the larger the
            // displacement from the reference, the faster we move - but
            // hard-capped by robot.max_lin_speed.
            double s = cfg_.position_scale;
            double vx = cfg_.sign_x * s * g.rightDeltaPosition[2] * -1.0; // Leap Z -> robot X (toward user = forward)
            double vy = cfg_.sign_y * s * g.rightDeltaPosition[0] * -1.0; // Leap X -> robot Y (left/right)
            double vz = cfg_.sign_z * s * g.rightDeltaPosition[1];        // Leap Y -> robot Z (up)
            out.linear_velocity = {vx, vy, vz};
            break;
        }
        case DemoState::OrientationControl: {
            out.ui_mode_text = "ORIENTATION MODE";
            out.ui_status_text = "Right-hand fist rotates the tool.";
            out.ui_prompt_text = "Open right hand to pause.";

            if (g.gripperGestureArmed) {
                enter(DemoState::Gripper, g, now_s, out);
                break;
            }
            if (!conditionsForOrientationControl(g)) {
                if (conditionsForRecenter(g)) {
                    enter(DemoState::Recenter, g, now_s, out);
                } else if (conditionsForPositionControl(g)) {
                    enter(DemoState::PositionControl, g, now_s, out);
                } else {
                    out.hardStop = true;
                }
                break;
            }
            double s = cfg_.orientation_scale;
            double wx = cfg_.sign_rx * s * g.rightDeltaOrientation[0];
            double wy = cfg_.sign_ry * s * g.rightDeltaOrientation[1];
            double wz = cfg_.sign_rz * s * g.rightDeltaOrientation[2];
            out.angular_velocity = {wx, wy, wz};
            break;
        }
        case DemoState::Recenter: {
            out.hardStop = true;
            out.ui_mode_text = "RECENTER";
            out.ui_status_text = "Motion paused.";
            out.ui_prompt_text = "Move your right hand, then close the fist to resume.";

            if (g.gripperGestureArmed) {
                enter(DemoState::Gripper, g, now_s, out);
                break;
            }
            // Return to the mode implied by the left hand.
            if (conditionsForPositionControl(g)) {
                enter(DemoState::PositionControl, g, now_s, out);
            } else if (conditionsForOrientationControl(g)) {
                enter(DemoState::OrientationControl, g, now_s, out);
            } else if (!g.leftPresent || !g.rightPresent) {
                // Hand disappeared while recentering -> Fault (Rule 4/5).
                fault_reason_ = g.leftPresent ? FaultReason::RightHandLost
                                              : FaultReason::LeftHandLost;
                enter(DemoState::Fault, g, now_s, out);
            }
            break;
        }
        case DemoState::Gripper: {
            out.hardStop = true;
            out.ui_mode_text = "GRIPPER";
            out.ui_status_text = "Two-hand gesture recognised.";
            out.ui_prompt_text = "Move hands together = close | apart = open.";

            if (g.gripperOpenImpulse) {
                out.openGripper = true;
                LOG_I("Gripper IMPULSE: OPEN (dist=%.1f mm)", g.handDistance_mm);
            } else if (g.gripperCloseImpulse) {
                out.closeGripper = true;
                LOG_I("Gripper IMPULSE: CLOSE (dist=%.1f mm)", g.handDistance_mm);
            }

            // Recommended default: on impulse, return to Ready so user
            // has to re-engage deliberately (Assumption E).
            if (out.openGripper || out.closeGripper) {
                enter(DemoState::Ready, g, now_s, out);
                break;
            }
            if (!g.gripperGestureArmed) {
                // User dropped the gripper posture without crossing a
                // threshold - back to Ready.
                enter(DemoState::Ready, g, now_s, out);
            }
            break;
        }
        case DemoState::Fault: {
            out.hardStop = true;
            out.ui_mode_text = "HOLD";
            out.ui_status_text = std::string("Hold: ") + faultReasonText(fault_reason_);
            out.ui_prompt_text = "Remove hands and show both again to re-engage.";

            // Clear fault when both hands have been removed and then
            // re-presented (i.e. deliberate re-engagement in Ready).
            bool clearable = g.sensor_ok &&
                             (fault_reason_ != FaultReason::SensorDisconnected);
            if (clearable && !g.leftPresent && !g.rightPresent) {
                fault_reason_ = FaultReason::None;
                enter(DemoState::Ready, g, now_s, out);
            } else if (fault_reason_ == FaultReason::SensorDisconnected && g.sensor_ok) {
                fault_reason_ = FaultReason::None;
                enter(DemoState::Ready, g, now_s, out);
            }
            break;
        }
        case DemoState::Idle:
            // Handled above.
            out.hardStop = true;
            break;
        }

        if (state_ == before) break; // no transition - second pass not useful.
    }

    return out;
}

} // namespace dgd

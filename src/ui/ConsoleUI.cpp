#include "ui/ConsoleUI.hpp"

#include <cstdio>

namespace dgd {

namespace {

const char* gripperText(IGripperController::State s) {
    switch (s) {
        case IGripperController::State::Open:    return "OPEN";
        case IGripperController::State::Closed:  return "CLOSED";
        case IGripperController::State::Unknown: return "?";
    }
    return "?";
}

const char* postureText(HandPosture p) {
    switch (p) {
        case HandPosture::Open:    return "open";
        case HandPosture::Closed:  return "closed";
        case HandPosture::Unknown: return "--";
    }
    return "?";
}

} // namespace

ConsoleUI::ConsoleUI(const Config& cfg) : cfg_(cfg) {}

void ConsoleUI::render(const Frame& f) {
    if (cfg_.ui_clear_each_tick) {
        // ANSI clear + home. Works on Windows 10+ and any POSIX terminal.
        std::printf("\x1b[2J\x1b[H");
    }

    std::printf("==============================================================\n");
    std::printf(" DOOSAN A0912 - Leap Motion gesture demo\n");
    std::printf("==============================================================\n");
    std::printf(" STATE   : %-12s    BUTTON: %s\n",
                stateName(f.state),
                f.button_active ? "ACTIVE" : "INACTIVE");
    std::printf(" MODE    : %s\n", f.mode_text.c_str());
    std::printf(" STATUS  : %s\n", f.status_text.c_str());
    std::printf(" PROMPT  : %s\n", f.prompt_text.c_str());
    if (f.fault != FaultReason::None) {
        std::printf(" FAULT   : %s\n", faultReasonText(f.fault));
    } else {
        std::printf(" FAULT   : -\n");
    }
    std::printf("--------------------------------------------------------------\n");
    std::printf(" SENSOR  : %s\n", f.sensor_ok ? "connected" : "NOT CONNECTED");
    std::printf(" LEFT    : %s (%s, c=%.2f)\n",
                f.gesture.leftPresent ? "present" : "missing",
                postureText(f.gesture.leftPosture), f.gesture.leftConfidence);
    std::printf(" RIGHT   : %s (%s, c=%.2f)\n",
                f.gesture.rightPresent ? "present" : "missing",
                postureText(f.gesture.rightPosture), f.gesture.rightConfidence);
    if (f.gesture.handDistance_mm >= 0.0) {
        std::printf(" HANDS   : %.1f mm apart%s\n",
                    f.gesture.handDistance_mm,
                    f.gesture.gripperGestureArmed ? "  [GRIPPER ARMED]" : "");
    } else {
        std::printf(" HANDS   : -\n");
    }
    std::printf(" GRIPPER : %s\n", gripperText(f.gripper));
    std::printf(" POSE    : X=%.1f Y=%.1f Z=%.1f  Rx=%.1f Ry=%.1f Rz=%.1f%s\n",
                f.pose.x, f.pose.y, f.pose.z,
                f.pose.rx, f.pose.ry, f.pose.rz,
                f.workspace_limit ? "   [LIMIT]" : "");

    if (cfg_.ui_show_hand_debug) {
        std::printf("--------------------------------------------------------------\n");
        std::printf(" Dbg dP  : (%.1f, %.1f, %.1f) mm\n",
                    f.gesture.rightDeltaPosition[0],
                    f.gesture.rightDeltaPosition[1],
                    f.gesture.rightDeltaPosition[2]);
        std::printf(" Dbg dR  : (%.1f, %.1f, %.1f) deg\n",
                    f.gesture.rightDeltaOrientation[0],
                    f.gesture.rightDeltaOrientation[1],
                    f.gesture.rightDeltaOrientation[2]);
    }
    std::printf("--------------------------------------------------------------\n");
    std::printf(" [SPACE] toggle authorisation    [Q/ESC] quit\n");
    std::printf("==============================================================\n");
    std::fflush(stdout);
    first_render_ = false;
}

} // namespace dgd

// Minimal smoke test for the state machine. Runs without any SDK.
// Not a full unit-test framework - just a handful of asserts that
// guarantee the major transitions still fire as expected.

#include "config/Config.hpp"
#include "gesture/GestureTypes.hpp"
#include "state/StateMachine.hpp"
#include "util/Logger.hpp"

#include <cassert>
#include <cstdio>

using namespace dgd;

static GestureReport makeReport(bool sensor, bool lp, HandPosture lpose,
                                bool rp, HandPosture rpose) {
    GestureReport r;
    r.sensor_ok = sensor;
    r.leftPresent = lp;
    r.leftPosture = lpose;
    r.leftConfidence = lp ? 1.0 : 0.0;
    r.rightPresent = rp;
    r.rightPosture = rpose;
    r.rightConfidence = rp ? 1.0 : 0.0;
    return r;
}

static void expectState(StateMachine& sm, DemoState expected, const char* label) {
    if (sm.state() != expected) {
        std::fprintf(stderr, "FAIL %s: expected %s, got %s\n",
                     label, stateName(expected), stateName(sm.state()));
        std::exit(1);
    }
    std::printf("  ok: %s -> %s\n", label, stateName(expected));
}

int main() {
    Logger::instance().configure(LogLevel::Warn, "");
    Config cfg;
    StateMachine sm(cfg);
    double t = 0.0;

    // Button off -> Idle, no motion.
    sm.step(makeReport(true, true, HandPosture::Open, true, HandPosture::Closed),
            /*button=*/false, t++);
    expectState(sm, DemoState::Idle, "button off");

    // Button on but no hands -> Ready.
    sm.step(makeReport(true, false, HandPosture::Unknown, false, HandPosture::Unknown),
            /*button=*/true, t++);
    expectState(sm, DemoState::Ready, "button on, no hands");

    // Valid position engagement -> PositionControl.
    sm.step(makeReport(true, true, HandPosture::Open, true, HandPosture::Closed),
            /*button=*/true, t++);
    expectState(sm, DemoState::PositionControl, "valid position engagement");

    // Right hand opens -> Recenter.
    sm.step(makeReport(true, true, HandPosture::Open, true, HandPosture::Open),
            /*button=*/true, t++);
    expectState(sm, DemoState::Recenter, "right hand opens");

    // Right hand closes again -> back to PositionControl.
    sm.step(makeReport(true, true, HandPosture::Open, true, HandPosture::Closed),
            /*button=*/true, t++);
    expectState(sm, DemoState::PositionControl, "re-engage from recenter");

    // Left hand disappears during control -> Fault.
    sm.step(makeReport(true, false, HandPosture::Unknown, true, HandPosture::Closed),
            /*button=*/true, t++);
    expectState(sm, DemoState::Fault, "left hand disappears");

    // Both hands removed -> Fault clears to Ready.
    sm.step(makeReport(true, false, HandPosture::Unknown, false, HandPosture::Unknown),
            /*button=*/true, t++);
    expectState(sm, DemoState::Ready, "fault recovers after both hands removed");

    // Orientation engagement: left closed + right closed.
    sm.step(makeReport(true, true, HandPosture::Closed, true, HandPosture::Closed),
            /*button=*/true, t++);
    expectState(sm, DemoState::OrientationControl, "valid orientation engagement");

    // Gripper gesture wins.
    GestureReport g = makeReport(true, true, HandPosture::Open, true, HandPosture::Closed);
    g.gripperGestureArmed = true;
    g.gripperOpenImpulse  = true;
    g.handDistance_mm = 200.0;
    sm.step(g, /*button=*/true, t++);
    expectState(sm, DemoState::Ready, "gripper impulse returns to Ready");

    // Button released -> Idle regardless of hand posture.
    sm.step(makeReport(true, true, HandPosture::Open, true, HandPosture::Closed),
            /*button=*/false, t++);
    expectState(sm, DemoState::Idle, "button released");

    // Sensor disconnected while authorised -> Fault with proper reason.
    sm.step(makeReport(false, false, HandPosture::Unknown, false, HandPosture::Unknown),
            /*button=*/true, t++);
    expectState(sm, DemoState::Fault, "sensor disconnected -> fault");

    std::printf("All state machine smoke tests passed.\n");
    return 0;
}

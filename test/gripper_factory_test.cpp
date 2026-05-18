// Unit test: gripper factory selection logic + no-op semantics.
//
// Acceptance check from the spec:
//   gripper_enabled = false      -> NoopGripper, no I/O, open/close are no-ops
//   gripper_enabled = true,      -> respect backend choice
//     backend = "none"           -> NoopGripper
//     backend = "tool_io"        -> ToolIoGripperController (legacy)
//     type    = "qb_softclaw" -> QbSoftClawGripper (stub)

#include "config/Config.hpp"
#include "gripper/GripperFactory.hpp"
#include "robot/IRobotController.hpp"
#include "robot/RobotPose.hpp"
#include "util/Logger.hpp"

#include <array>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

using namespace dgd;

namespace {
// Stub robot controller: never connects, never moves. The gripper factory
// only forwards the reference to backends that may use it (tool_io); we
// only need it to satisfy the signature.
class StubRobot : public IRobotController {
public:
    bool connect(const std::string&, int, double) override { return true; }
    void disconnect() override {}
    bool isConnected() const override { return true; }
    bool engage() override { return true; }
    void disengage() override {}
    bool moveHome(const RobotPose&) override { return true; }
    bool sendCartesianVelocity(const std::array<double, 6>&) override { return true; }
    void stopMotion() override {}
    void emergencyStop() override {}
    bool sendCartesianMicroMove(const RobotPose&, double, double, double, double,
                                double) override { return true; }
    bool getCurrentPose(RobotPose&) override { return true; }
    std::string lastError() const override { return ""; }
};

void expect(bool cond, const char* label) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", label);
        std::exit(1);
    }
    std::printf("  ok: %s\n", label);
}
} // namespace

int main() {
    Logger::instance().configure(LogLevel::Warn, "");
    StubRobot robot;

    // 1. Disabled -> NoopGripper, open/close are no-ops returning true.
    {
        Config cfg;
        cfg.gripper_enabled = false;
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g), "disabled: factory returns a gripper");
        expect(g->connect(), "disabled: connect() succeeds");
        expect(!g->isConnected(), "disabled: isConnected() == false (no I/O)");
        expect(g->open(),  "disabled: open()  returns true");
        expect(g->close(), "disabled: close() returns true");
        expect(g->lastCommandedState() == IGripperController::State::Closed,
               "disabled: last state tracked");
        expect(!g->isAvailable(), "disabled: isAvailable() == false");
    }

    // 2. enabled = true with backend = "none" -> NoopGripper (same as above).
    {
        Config cfg;
        cfg.gripper_enabled = true;
        cfg.gripper_backend = "none";
        auto g = makeGripper(cfg, robot);
        expect(!g->isAvailable(),
               "enabled+backend=none: still NoopGripper (no I/O)");
    }

    // 3. qb_softclaw stub: connects but explicitly logs as stub.
    {
        Config cfg;
        cfg.gripper_enabled = true;
        cfg.gripper_type    = "qb_softclaw";
        cfg.gripper_backend = "qb_softclaw";
        auto g = makeGripper(cfg, robot);
        expect(g->connect(), "qb stub: connect() succeeds (no SDK linked)");
        expect(g->isAvailable(), "qb stub: isAvailable() true after connect");
        expect(g->open() && g->close(), "qb stub: open/close return true");
        expect(g->setClosure(50.0),
               "qb stub: setClosure(50%) returns true (logging-only)");
        expect(g->setClosure(-10.0),
               "qb stub: out-of-range closure clamped, returns true");
        expect(g->setClosure(200.0),
               "qb stub: closure clamped above 100, returns true");
    }

    // 4. Unknown backend falls back to NoopGripper.
    {
        Config cfg;
        cfg.gripper_enabled = true;
        cfg.gripper_backend = "this_backend_does_not_exist";
        cfg.gripper_type    = "unknown";
        auto g = makeGripper(cfg, robot);
        expect(!g->isAvailable(),
               "unknown backend: fell back to NoopGripper");
    }

    std::printf("All gripper factory tests passed.\n");
    return 0;
}

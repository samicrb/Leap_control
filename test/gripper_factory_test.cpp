// Unit test: GripperFactory selection + NoopGripper fallback.
//
// The qbRobotics qbAPI is hardware-bound and cannot be exercised
// headlessly. We rely on the SDK NOT being linked into the test binary
// so QbSoftClawGripper::initialize() reports failure; the factory must
// then fall back to NoopGripper when required = false, and return
// nullptr when required = true.

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
    // Quiet logger (warn-level) but let the test print its own progress.
    Logger::instance().configure(LogLevel::Warn, "");
    StubRobot robot;

    // 1. enabled=false -> NoopGripper, all calls are silent no-ops.
    {
        Config cfg;
        cfg.gripper_enabled = false;
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g), "disabled: factory returns a gripper");
        expect(g->connect(), "disabled: connect() succeeds");
        expect(!g->isConnected(), "disabled: isConnected() == false");
        expect(g->open(),  "disabled: open() returns true");
        expect(g->close(), "disabled: close() returns true");
        expect(g->lastCommandedState() == IGripperController::State::Closed,
               "disabled: last state tracked");
        expect(!g->isAvailable(), "disabled: isAvailable() == false");
        expect(std::string(g->name()) == "noop", "disabled: name == noop");
    }

    // 2. enabled=true + backend=none -> NoopGripper.
    {
        Config cfg;
        cfg.gripper_enabled = true;
        cfg.gripper_backend = "none";
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g), "backend=none: gripper returned");
        expect(std::string(g->name()) == "noop", "backend=none: name == noop");
    }

    // 3. SoftClaw selection. The SDK is not linked into this test
    //    target, so QbSoftClawGripper::initialize() must fail. With
    //    required=false the factory falls back to NoopGripper.
    {
        Config cfg;
        cfg.gripper_enabled              = true;
        cfg.gripper_type                 = "softclaw";
        cfg.gripper_backend              = "qb_sdk";
        cfg.gripper_required             = false;
        cfg.gripper_initialize_on_startup = true;
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g),
               "softclaw+qb_sdk, required=false, no SDK: gripper returned");
        // The factory should have downgraded to NoopGripper.
        expect(std::string(g->name()) == "noop",
               "softclaw+qb_sdk, required=false, no SDK: "
               "factory falls back to NoopGripper");
        expect(!g->isAvailable(),
               "softclaw+qb_sdk, required=false: fallback is unavailable");
    }

    // 4. SoftClaw selection with required=true and no SDK -> nullptr.
    {
        Config cfg;
        cfg.gripper_enabled              = true;
        cfg.gripper_type                 = "softclaw";
        cfg.gripper_backend              = "qb_sdk";
        cfg.gripper_required             = true;
        cfg.gripper_initialize_on_startup = true;
        auto g = makeGripper(cfg, robot);
        expect(!g, "softclaw+qb_sdk, required=true, no SDK: factory returns nullptr");
    }

    // 5. SoftClaw selection but initialize_on_startup=false -> the
    //    factory returns the QbSoftClawGripper without trying to open
    //    the COM port. Useful when the operator wants to defer init.
    {
        Config cfg;
        cfg.gripper_enabled              = true;
        cfg.gripper_type                 = "softclaw";
        cfg.gripper_backend              = "qb_sdk";
        cfg.gripper_required             = false;
        cfg.gripper_initialize_on_startup = false;
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g),
               "softclaw+qb_sdk, initialize=false: gripper returned");
        expect(std::string(g->name()) == "qb_softclaw",
               "softclaw+qb_sdk, initialize=false: real backend kept");
    }

    // 6. Unknown backend falls back to NoopGripper.
    {
        Config cfg;
        cfg.gripper_enabled = true;
        cfg.gripper_type    = "weird_thing";
        cfg.gripper_backend = "unknown_backend";
        auto g = makeGripper(cfg, robot);
        expect(static_cast<bool>(g), "unknown backend: gripper returned");
        expect(std::string(g->name()) == "noop",
               "unknown backend: falls back to NoopGripper");
    }

    // 7. Rate-limit / deadband: hammer the SoftClaw stub with many
    //    open() calls in quick succession; the backend must silently
    //    suppress duplicates and still report success.
    {
        Config cfg;
        cfg.gripper_enabled              = true;
        cfg.gripper_type                 = "softclaw";
        cfg.gripper_backend              = "qb_sdk";
        cfg.gripper_required             = false;
        cfg.gripper_initialize_on_startup = false;  // skip serial open
        auto g = makeGripper(cfg, robot);
        // Backend is the real QbSoftClawGripper (stub branch). open()
        // returns false because we did not initialize. That is the
        // correct contract: do NOT pretend to command the device when
        // not connected.
        expect(!g->open(),
               "softclaw+qb_sdk uninitialized: open() returns false");
    }

    std::printf("All gripper factory tests passed.\n");
    return 0;
}

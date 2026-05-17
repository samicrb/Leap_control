#pragma once

#include "motion/IMotionBackend.hpp"

namespace dgd {

// AmovelMotionBackend - thin façade for the existing amovel pursuit
// pipeline.
//
// The body of the amovel emission logic still lives inside
// Application::tick() to avoid a large invasive refactor. This class
// exists so:
//   1. The factory in Application can return a uniform IMotionBackend
//      pointer for whichever backend was selected.
//   2. The event log can clearly say which backend is active.
//   3. A future commit can finish lifting the amovel emission state
//      (filtered_velocity_, ramp_to_zero_, last_emitted_step_*, etc.)
//      into the backend class itself.
//
// In its current shape onActiveEntry / onActiveExit / onTick are no-op
// hooks. Application gates its existing amovel code on
// (backend_kind == Amovel) and runs it inline as before.
class AmovelMotionBackend final : public IMotionBackend {
public:
    const char* name() const override { return "amovel"; }
    MotionBackendKind kind() const override { return MotionBackendKind::Amovel; }

    void attach(const Config& /*cfg*/, IRobotController& /*robot*/) override {}
    void onActiveEntry(const RobotPose& /*seed*/, double /*now_s*/) override {}
    void onActiveExit(double /*now_s*/) override {}
    void onReanchor(const RobotPose& /*pose*/, double /*now_s*/) override {}

    // Application::tick runs the amovel path inline and fills its own
    // MotionTickResult locally. This shim returns an empty result and
    // is never actually called when backend_kind == Amovel (see the
    // dispatch in Application).
    MotionTickResult onTick(const MotionTickContext& /*ctx*/) override {
        return MotionTickResult{};
    }

    std::optional<RobotPose> currentTarget() const override { return std::nullopt; }
};

} // namespace dgd

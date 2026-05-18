#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"
#include "robot/IRobotController.hpp"

#include <memory>

namespace dgd {

// Selects a gripper implementation based on [gripper] in demo_config.ini:
//
//   enabled = false              -> NoopGripper
//   type = softclaw / qb_softclaw
//     backend = qb_sdk           -> QbSoftClawGripper (qbRobotics SDK)
//     backend = none             -> NoopGripper
//   backend = tool_io            -> ToolIoGripperController (legacy)
//   anything else                -> NoopGripper + warning
//
// If type == softclaw / qb_softclaw, backend == qb_sdk AND the
// SoftClaw fails to initialize:
//   - required = true  -> the factory returns nullptr (caller aborts).
//   - required = false -> the factory falls back to NoopGripper with a
//                         clear WARN log so the rest of the demo can
//                         continue running without the gripper.
//
// The function never throws.
std::unique_ptr<IGripperController> makeGripper(const Config& cfg,
                                                IRobotController& robot);

} // namespace dgd

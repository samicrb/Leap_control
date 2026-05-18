#pragma once

#include "config/Config.hpp"
#include "gripper/IGripperController.hpp"
#include "robot/IRobotController.hpp"

#include <memory>

namespace dgd {

// Selects a gripper implementation based on cfg.gripper_enabled,
// cfg.gripper_backend and cfg.gripper_type:
//
//   gripper_enabled = false         -> NoopGripper
//   backend = "none"                -> NoopGripper
//   backend = "tool_io"             -> ToolIoGripperController (legacy)
//   type    = "qb_softclaw"         -> QbSoftClawGripper (stub)
//   anything else                   -> NoopGripper + warning
//
// The function never throws and always returns a usable pointer so
// Application can call open() / close() unconditionally.
std::unique_ptr<IGripperController> makeGripper(const Config& cfg,
                                                IRobotController& robot);

} // namespace dgd

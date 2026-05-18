#include "gripper/GripperFactory.hpp"

#include "gripper/NoopGripper.hpp"
#include "gripper/QbSoftClawGripper.hpp"
#include "gripper/ToolIoGripperController.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace dgd {

namespace {
std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}
} // namespace

std::unique_ptr<IGripperController> makeGripper(const Config& cfg,
                                                IRobotController& robot) {
    if (!cfg.gripper_enabled) {
        LOG_I("Gripper selection: enabled=false -> NoopGripper.");
        return std::make_unique<NoopGripper>();
    }

    const std::string type    = lower(cfg.gripper_type);
    const std::string backend = lower(cfg.gripper_backend);

    LOG_I("Gripper selection: type='%s' backend='%s' "
          "(port='%s' device_id=%d required=%s).",
          cfg.gripper_type.c_str(), cfg.gripper_backend.c_str(),
          cfg.gripper_port.c_str(), cfg.gripper_device_id,
          cfg.gripper_required ? "true" : "false");

    if (backend == "none") {
        LOG_I("Gripper selection: backend=none -> NoopGripper "
              "(type='%s' announced but no I/O backend selected).",
              cfg.gripper_type.c_str());
        return std::make_unique<NoopGripper>();
    }
    if (backend == "tool_io") {
        LOG_I("Gripper selection: backend=tool_io -> "
              "ToolIoGripperController (open DO=%d, close DO=%d).",
              cfg.gripper_open_do_index, cfg.gripper_close_do_index);
        return std::make_unique<ToolIoGripperController>(cfg, robot);
    }
    if (backend == "qb_sdk" &&
        (type == "softclaw" || type == "qb_softclaw")) {
        LOG_I("Gripper selection: PC-side qbRobotics SoftClaw via qbAPI.");
        auto g = std::make_unique<QbSoftClawGripper>(cfg);
        if (cfg.gripper_initialize_on_startup) {
            if (!g->initialize()) {
                if (cfg.gripper_required) {
                    LOG_E("Gripper: SoftClaw initialization failed AND "
                          "[gripper].required=true. Aborting startup.");
                    return nullptr;
                }
                LOG_W("Gripper: SoftClaw initialization failed; "
                      "[gripper].required=false -> falling back to "
                      "NoopGripper. (%s)",
                      g->lastError().c_str());
                return std::make_unique<NoopGripper>();
            }
        }
        return g;
    }

    LOG_W("Gripper selection: unknown type='%s' / backend='%s' -> "
          "falling back to NoopGripper.",
          cfg.gripper_type.c_str(), cfg.gripper_backend.c_str());
    return std::make_unique<NoopGripper>();
}

} // namespace dgd

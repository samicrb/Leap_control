#include "gripper/NoopGripper.hpp"

#include "util/Logger.hpp"

namespace dgd {

bool NoopGripper::connect() {
    LOG_I("Gripper: disabled (NoopGripper) - open/close calls are no-ops.");
    return true;
}

} // namespace dgd

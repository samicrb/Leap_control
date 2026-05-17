#include "gripper/QbSoftHandIndustryGripper.hpp"

#include "util/Logger.hpp"

namespace dgd {

QbSoftHandIndustryGripper::QbSoftHandIndustryGripper(const Config& cfg)
    : cfg_(cfg) {}

bool QbSoftHandIndustryGripper::connect() {
    // No vendor SDK is linked. We "succeed" so the Application can run,
    // and we log enough context that the operator knows the gripper is
    // a stub today.
    LOG_W("Gripper: QbSoftHandIndustry STUB backend "
          "(target=%s:%d/api - not yet wired). Commands will be logged "
          "but no I/O is performed.",
          cfg_.gripper_ip.c_str(), 0);
    LOG_I("  type=%s backend=%s open=%.1f close=%.1f pregrasp=%.1f "
          "speed=%.0f%% force=%.0f%% timeout=%.2fs wait_for_target=%d",
          cfg_.gripper_type.c_str(), cfg_.gripper_backend.c_str(),
          cfg_.gripper_open_position, cfg_.gripper_close_position,
          cfg_.gripper_pregrasp_position,
          cfg_.gripper_close_speed_percent, cfg_.gripper_close_force_percent,
          cfg_.gripper_command_timeout_s,
          cfg_.gripper_wait_for_target ? 1 : 0);
    connected_.store(true);
    return true;
}

void QbSoftHandIndustryGripper::disconnect() {
    if (connected_.load()) {
        LOG_I("Gripper: QbSoftHandIndustry stub disconnected.");
    }
    connected_.store(false);
}

bool QbSoftHandIndustryGripper::open() {
    LOG_I("Gripper [STUB qb]: OPEN  (target=%.1f, %s)",
          cfg_.gripper_open_position,
          cfg_.gripper_wait_for_target ? "wait" : "fire-and-forget");
    last_state_ = State::Open;
    return true;
}

bool QbSoftHandIndustryGripper::close() {
    LOG_I("Gripper [STUB qb]: CLOSE (target=%.1f, speed=%.0f%%, force=%.0f%%)",
          cfg_.gripper_close_position,
          cfg_.gripper_close_speed_percent,
          cfg_.gripper_close_force_percent);
    last_state_ = State::Closed;
    return true;
}

bool QbSoftHandIndustryGripper::setClosure(double percent) {
    if (percent < 0.0)   percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    LOG_I("Gripper [STUB qb]: setClosure(%.1f%%)", percent);
    if      (percent < 5.0)  last_state_ = State::Open;
    else if (percent > 95.0) last_state_ = State::Closed;
    return true;
}

void QbSoftHandIndustryGripper::stopOrHold() {
    LOG_I("Gripper [STUB qb]: stopOrHold");
}

} // namespace dgd

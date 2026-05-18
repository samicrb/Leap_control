// QbSoftClawGripper.cpp
//
// PC-side qbRobotics SoftClaw integration via the official qbAPI
// (https://github.com/NMMI/qbAPI). All Doosan / robot-controller-side
// gripper handling is intentionally absent from this file.
//
// SDK integration is gated by HAVE_QBROBOTICS_SDK. When the SDK header
// `qbmove_communications.h` is available the real serial calls are
// compiled in; otherwise the methods log their intent and report the
// device as unavailable (the GripperFactory then falls back to
// NoopGripper if required=false).

#include "gripper/QbSoftClawGripper.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#if defined(HAVE_QBROBOTICS_SDK) && HAVE_QBROBOTICS_SDK
  // qbAPI C/C++ header from https://github.com/NMMI/qbAPI .
  // The header is intentionally included only here - the rest of the
  // codebase never sees qb types.
  #ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4244 4267)
  #endif
  #include "qbmove_communications.h"
  #ifdef _MSC_VER
    #pragma warning(pop)
  #endif
#endif

namespace dgd {

struct QbSoftClawGripper::Impl {
#if defined(HAVE_QBROBOTICS_SDK) && HAVE_QBROBOTICS_SDK
    comm_settings comm{};
    bool          comm_open = false;
#endif
    // Stash a stub-mode flag so disconnect() can be idempotent.
    bool stub_logged = false;
};

double QbSoftClawGripper::nowMonotonic() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

QbSoftClawGripper::QbSoftClawGripper(const Config& cfg)
    : cfg_(cfg), p_(std::make_unique<Impl>()) {}

QbSoftClawGripper::~QbSoftClawGripper() {
    if (connected_.load()) {
        disconnect();
    }
}

bool QbSoftClawGripper::initialize() {
    return connect();
}

bool QbSoftClawGripper::connect() {
    LOG_I("Gripper: connecting to qb SoftClaw "
          "(port=%s, device_id=%d, baudrate=%d).",
          cfg_.gripper_port.c_str(),
          cfg_.gripper_device_id,
          cfg_.gripper_baudrate);

#if !defined(HAVE_QBROBOTICS_SDK) || !HAVE_QBROBOTICS_SDK
    // SDK not linked. We make this VERY obvious in the log so an
    // operator does not assume the SoftClaw is wired when it is not.
    last_error_ =
        "qbRobotics SDK not linked at build time (HAVE_QBROBOTICS_SDK=0). "
        "Drop the NMMI/qbAPI sources under external/qbrobotics_sdk and "
        "rebuild, or point QBROBOTICS_SDK_ROOT at the SDK. See "
        "docs/SOFTCLAW_INTEGRATION.md.";
    LOG_W("Gripper: %s", last_error_.c_str());
    connected_.store(false);
    return false;
#else
    if (cfg_.dryrun_gripper) {
        LOG_W("Gripper: dryrun.gripper=true - skipping real qbAPI open.");
        connected_.store(true);
        return true;
    }

    // openRS485 takes (handle*, port, baud). Windows + Linux signatures
    // both accept int baud; on Windows the qbAPI default is 2 000 000.
    openRS485(&p_->comm, cfg_.gripper_port.c_str(), cfg_.gripper_baudrate);

#if defined(_WIN32)
    if (p_->comm.file_handle == INVALID_HANDLE_VALUE) {
        last_error_ = std::string("openRS485 failed on port ") +
                      cfg_.gripper_port +
                      " (check Device Manager for the qbRobotics USB / "
                      "RS-485 adapter, FTDI drivers, and that the port "
                      "is not held by another process).";
        LOG_E("Gripper: %s", last_error_.c_str());
        return false;
    }
#else
    if (p_->comm.file_handle < 0) {
        last_error_ = std::string("openRS485 failed on port ") +
                      cfg_.gripper_port;
        LOG_E("Gripper: %s", last_error_.c_str());
        return false;
    }
#endif
    p_->comm_open = true;

    // Ping the configured device id. If the SoftClaw is not on the bus
    // we abort cleanly; the factory will fall back to NoopGripper when
    // required=false.
    if (commPing(&p_->comm, cfg_.gripper_device_id) < 0) {
        last_error_ =
            "qbAPI commPing returned no response for device id " +
            std::to_string(cfg_.gripper_device_id) +
            " on " + cfg_.gripper_port +
            ". Verify the SoftClaw is powered and the ID matches.";
        LOG_E("Gripper: %s", last_error_.c_str());
        closeRS485(&p_->comm);
        p_->comm_open = false;
        return false;
    }

    // Activate the motor. The qbAPI activate flag is non-zero -> ON.
    commActivate(&p_->comm, cfg_.gripper_device_id, 1);

    LOG_I("Gripper: qb SoftClaw connected and activated "
          "(device id %d on %s).",
          cfg_.gripper_device_id, cfg_.gripper_port.c_str());
    connected_.store(true);
    return true;
#endif
}

void QbSoftClawGripper::disconnect() {
    if (!connected_.exchange(false)) return;

#if defined(HAVE_QBROBOTICS_SDK) && HAVE_QBROBOTICS_SDK
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper: qb SoftClaw dryrun-disconnect.");
        return;
    }
    if (p_ && p_->comm_open) {
        // Best-effort deactivate then close the port.
        commActivate(&p_->comm, cfg_.gripper_device_id, 0);
        closeRS485(&p_->comm);
        p_->comm_open = false;
    }
    LOG_I("Gripper: qb SoftClaw disconnected.");
#else
    LOG_I("Gripper: qb SoftClaw stub disconnected.");
#endif
}

// Translate the high-level intent open/close into a (position, deflection)
// pair from the config, then forward through the rate-limit + deadband.
bool QbSoftClawGripper::open() {
    last_state_ = State::Open;
    return sendInputs(cfg_.gripper_open_position,
                      cfg_.gripper_open_deflection,
                      "OPEN");
}

bool QbSoftClawGripper::close() {
    last_state_ = State::Closed;
    return sendInputs(cfg_.gripper_close_position,
                      cfg_.gripper_close_deflection,
                      "CLOSE");
}

bool QbSoftClawGripper::setClosurePercent(double percent) {
    if (percent < 0.0)   percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    const double t = percent / 100.0;
    const int pos = static_cast<int>(std::round(
        cfg_.gripper_open_position +
        t * (cfg_.gripper_close_position - cfg_.gripper_open_position)));
    const int def = static_cast<int>(std::round(
        cfg_.gripper_open_deflection +
        t * (cfg_.gripper_close_deflection - cfg_.gripper_open_deflection)));
    if      (percent < 5.0)  last_state_ = State::Open;
    else if (percent > 95.0) last_state_ = State::Closed;
    return sendInputs(pos, def, "SETCLOSURE");
}

void QbSoftClawGripper::stop() {
    if (!connected_.load()) return;
#if defined(HAVE_QBROBOTICS_SDK) && HAVE_QBROBOTICS_SDK
    if (cfg_.dryrun_gripper) {
        LOG_I("Gripper: stop() dryrun.");
        return;
    }
    // Deactivate the motor. The SoftClaw will release without holding
    // torque - safer than leaving it grasping at full stiffness on
    // application exit.
    commActivate(&p_->comm, cfg_.gripper_device_id, 0);
    LOG_I("Gripper: qb SoftClaw stop / deactivate.");
#else
    LOG_I("Gripper [STUB qb_softclaw]: stop / deactivate.");
#endif
}

bool QbSoftClawGripper::rateLimit(int position, int deflection, double now_s) {
    // Period gate.
    if (last_command_s_ >= 0.0 && cfg_.gripper_min_command_period_ms > 0) {
        const double min_dt = cfg_.gripper_min_command_period_ms / 1000.0;
        if ((now_s - last_command_s_) < min_dt) {
            return false;
        }
    }
    // Deadband on input change. The deadband is the minimum absolute
    // change in EITHER input that earns a fresh serial transmission.
    if (last_position_ >= 0 && cfg_.gripper_command_deadband > 0) {
        const int dpos = std::abs(position   - last_position_);
        const int ddef = std::abs(deflection - last_deflection_);
        if (dpos < cfg_.gripper_command_deadband &&
            ddef < cfg_.gripper_command_deadband) {
            return false;
        }
    }
    return true;
}

bool QbSoftClawGripper::sendInputs(int position, int deflection,
                                   const char* tag) {
    if (!connected_.load()) {
        last_error_ = "Gripper not connected; ignoring command.";
        return false;
    }
    const double now_s = nowMonotonic();
    if (!rateLimit(position, deflection, now_s)) {
        // Silent skip - this is the common case during continuous control.
        return true;
    }

#if defined(HAVE_QBROBOTICS_SDK) && HAVE_QBROBOTICS_SDK
    if (!cfg_.dryrun_gripper) {
        short int inputs[2] = {
            static_cast<short int>(position),
            static_cast<short int>(deflection),
        };
        commSetInputs(&p_->comm, cfg_.gripper_device_id, inputs);
    }
    LOG_I("Gripper [qb_softclaw]: %s pos=%d def=%d (port=%s id=%d)",
          tag, position, deflection,
          cfg_.gripper_port.c_str(), cfg_.gripper_device_id);
#else
    LOG_I("Gripper [STUB qb_softclaw]: %s pos=%d def=%d (port=%s id=%d) - "
          "SDK not linked, no serial I/O.",
          tag, position, deflection,
          cfg_.gripper_port.c_str(), cfg_.gripper_device_id);
#endif

    last_command_s_  = now_s;
    last_position_   = position;
    last_deflection_ = deflection;
    return true;
}

} // namespace dgd

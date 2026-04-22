#pragma once

#include "robot/RobotPose.hpp"

#include <array>
#include <string>

namespace dgd {

// Robot controller abstraction. All DRFL symbols live in the adapter
// implementation; the rest of the codebase never includes DRFL headers.
class IRobotController {
public:
    virtual ~IRobotController() = default;

    // --- Lifecycle -----------------------------------------------------
    // Connect to the robot, switch to the mode used by the demo.
    // Blocks up to timeout_s seconds. Returns true on success.
    virtual bool connect(const std::string& ip, int port, double timeout_s) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // Put the robot in a teleop-compatible state (servo on, manual mode).
    virtual bool engage() = 0;
    virtual void disengage() = 0;

    // Move to a known safe pose using a normal planned movel (used at
    // startup and after a fault). BLOCKING.
    virtual bool moveHome(const RobotPose& safe) = 0;

    // --- Streaming control ---------------------------------------------
    // Request a Cartesian velocity (mm/s, deg/s) in the base frame.
    // Adapter is responsible for forwarding this to the robot's
    // realtime streaming primitive at the correct rate.
    virtual bool sendCartesianVelocity(const std::array<double, 6>& twist) = 0;

    // Halt current motion immediately and safely. Must not throw.
    virtual void stopMotion() = 0;

    // --- State queries -------------------------------------------------
    virtual bool getCurrentPose(RobotPose& out) = 0;
    virtual std::string lastError() const = 0;
};

} // namespace dgd

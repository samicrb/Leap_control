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

    // SOFT pause. Stream a zero Cartesian velocity so the controller
    // decelerates smoothly to a stand-still and holds position.
    // Idempotent and safe to call at the demo loop rate (60 Hz) -
    // notably from passive states (IDLE / READY / RECENTER / GRIPPER).
    // Must not throw.
    virtual void stopMotion() = 0;

    // Discrete, NON-BLOCKING, NON-BLENDED Cartesian micro-motion to a
    // target pose. The adapter MUST use amovel(...) with radius=0 (or an
    // equivalent non-blending overload) and MUST NOT call mwait().
    //
    // This is the only motion primitive the Application's active loop is
    // allowed to use during PositionControl / OrientationControl. The
    // scheduler (Application) caps the issue rate (typically 5 Hz) and
    // bounds the per-command delta so each motion completes well before
    // the next is issued.
    //
    // vel/acc are scalar caps (mm/s, deg/s, mm/s^2, deg/s^2). Returns
    // false if the command was refused / the link isn't ready.
    virtual bool sendCartesianMicroMove(const RobotPose& target,
                                        double lin_vel, double ang_vel,
                                        double lin_acc, double ang_acc,
                                        double blend_radius_mm = 0.0,
                                        const std::string& blend_type = "duplicate") = 0;

    // HARD halt. Issue a controller-level STOP_TYPE_QUICK. Reserved for
    // real faults and shutdown: this can drop the servo into SAFE_OFF
    // on some controllers, so do NOT call it from a streaming loop.
    // Must not throw.
    virtual void emergencyStop() = 0;

    // --- State queries -------------------------------------------------
    virtual bool getCurrentPose(RobotPose& out) = 0;
    virtual std::string lastError() const = 0;
};

} // namespace dgd

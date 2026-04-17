#pragma once

#include <string>

namespace dgd {

// Abstract gripper interface. Impulse-style API on purpose:
// Application sends a single open() / close() when the gesture layer
// produces an impulse event. Continuous-command behaviour must not be
// supported by any implementation (Rule 7 of the context).
class IGripperController {
public:
    virtual ~IGripperController() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    virtual bool open()  = 0;
    virtual bool close() = 0;

    // Last physical state we commanded (not necessarily measured).
    enum class State { Unknown, Open, Closed };
    virtual State lastCommandedState() const = 0;

    virtual std::string lastError() const = 0;
};

} // namespace dgd

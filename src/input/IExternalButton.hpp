#pragma once

namespace dgd {

// Source of the external supervisor button.
// Implementations: KeyboardButton (SPACE toggles), and optionally a
// GPIO/USB variant for the real event button.
class IExternalButton {
public:
    virtual ~IExternalButton() = default;

    virtual bool start() = 0;
    virtual void stop()  = 0;

    // Non-blocking. Returns true if the demo is currently authorised.
    virtual bool isActive() const = 0;
};

} // namespace dgd

#pragma once

#include "sensor/HandFrame.hpp"

namespace dgd {

// Sensor abstraction. Implementations:
//   - LeapSource (real LeapC / Gemini V5.x) - production path on event PC.
//   - LeapStubSource (always returns empty frame) - enabled when the
//     LeapC SDK is not present at build time, used for CI / smoke tests.
class ILeapSource {
public:
    virtual ~ILeapSource() = default;

    // Blocking or non-blocking init. Returns true if the sensor is ready.
    virtual bool start() = 0;
    virtual void stop()  = 0;

    // Non-blocking snapshot of the latest frame. Returns false if no new
    // frame has arrived since the last call (caller should keep the
    // previous frame but treat it as stale after hand_loss_timeout).
    virtual bool pollLatest(HandFrame& out) = 0;

    virtual bool isConnected() const = 0;
};

} // namespace dgd

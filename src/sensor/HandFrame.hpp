#pragma once

// POD types describing one tracked frame from the Leap Motion.
// Kept API-agnostic so the rest of the code never includes LeapC.h.

#include <array>
#include <cstdint>
#include <optional>

namespace dgd {

struct HandSample {
    // All positions in millimetres, expressed in the Leap device frame
    // (X right, Y up, Z toward user). Adapter converts once on ingest.
    std::array<double, 3> palm_position  {0.0, 0.0, 0.0};
    // Palm normal unit vector (Leap frame). Points "out of the palm".
    std::array<double, 3> palm_normal    {0.0, -1.0, 0.0};
    // Hand forward vector (from wrist toward fingers), unit length.
    std::array<double, 3> palm_direction {0.0, 0.0, -1.0};
    // Leap grab strength [0..1]: 0 = open, 1 = closed fist.
    double grab_strength = 0.0;
    // Leap confidence [0..1] - some versions expose pinch_strength instead;
    // we keep a generic confidence field filled by the adapter.
    double confidence    = 0.0;
    // Monotonic timestamp of this sample (seconds since app start).
    double timestamp_s   = 0.0;
};

// A single acquired frame. Either hand may be missing.
struct HandFrame {
    double timestamp_s = 0.0;
    std::optional<HandSample> left;
    std::optional<HandSample> right;
    // Whether the sensor is currently producing tracking data at all.
    bool sensor_connected = false;
    // Device id / serial - informational.
    std::uint64_t device_id = 0;
};

} // namespace dgd

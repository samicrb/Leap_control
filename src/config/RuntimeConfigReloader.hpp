#pragma once

// RuntimeConfigReloader - watches demo_config.ini and applies safe edits
// while the program is running.
//
// Design notes
// ------------
// The robot control pipeline reads Config fields by reference at every
// tick. We can therefore mutate the live Config struct in place from
// the same thread that drives the tick loop, between two ticks, without
// any synchronisation primitives. That's exactly what poll() does:
// when the .ini's mtime changes, parse the file into a fresh Config,
// diff field by field against the live one, validate each candidate
// change, and copy the validated ones back into the live struct.
//
// Critical constraints (per CLAUDE_HANDOFF.md, must not break):
//   - the active reference pose, the desired_target, and the last-
//     commanded target ARE NOT TOUCHED here (they live in Application).
//   - only WHITELISTED parameters can be mutated at runtime; everything
//     else stays at its boot-time value.
//   - invalid values are REJECTED. The previous live value is kept.
//   - file I/O for the watcher uses std::filesystem only; no blocking
//     calls inside the time-critical robot path.

#include <functional>
#include <string>

namespace dgd {

struct Config;
class EventLogger;

class RuntimeConfigReloader {
public:
    using EventSink = std::function<void(const std::string& message)>;

    RuntimeConfigReloader() = default;

    // Attach to a live Config. The reference must outlive this reloader.
    // event_sink is called for every applied / rejected change so the
    // caller can route into EventLogger / stderr / both.
    void attach(const std::string& config_path, Config& live, EventSink sink);
    void detach();

    // Cheap call from the tick loop. Reads mtime, only reparses when it
    // changes. Returns true if a reload was performed this call.
    bool poll(double now_s);

    // Force an immediate reload regardless of mtime.
    bool reloadNow();

    bool isAttached() const { return live_ != nullptr; }

private:
    bool applyDiff(const Config& fresh);

    std::string config_path_;
    Config*     live_ = nullptr;
    EventSink   sink_;

    // mtime as time-since-epoch in nanoseconds. Avoids std::filesystem
    // file_time_type comparison subtleties across platforms.
    long long   last_mtime_ns_ = 0;
    double      last_poll_s_   = 0.0;
};

} // namespace dgd

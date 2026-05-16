#pragma once

// EventLogger - timestamped human-readable event stream.
//
// Used to mark anything that isn't a numeric motion sample:
//   - program start / shutdown
//   - tracking lost / recovered
//   - deadman pressed / released
//   - state transitions
//   - robot command errors
//   - runtime config reloads and per-parameter changes
//
// Lines are written eagerly (one fflush per event) so the file remains
// useful even if the process is killed.

#include <cstdio>
#include <string>

namespace dgd {

struct Config;

class EventLogger {
public:
    EventLogger() = default;
    ~EventLogger();

    EventLogger(const EventLogger&) = delete;
    EventLogger& operator=(const EventLogger&) = delete;

    bool open(const Config& cfg);
    void close();
    bool isOpen() const { return file_ != nullptr; }
    const std::string& path() const { return path_; }

    // Write a single event line with current wall-clock timestamp.
    // Safe to call when not open: becomes a no-op (only console mirror
    // when also_console = true).
    void event(const std::string& message, bool also_console = false);

private:
    std::FILE*  file_ = nullptr;
    std::string path_;
    bool        warned_failure_ = false;
};

} // namespace dgd

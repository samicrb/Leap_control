#include "logging/EventLogger.hpp"

#include "config/Config.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace dgd {

namespace {

std::string fileStamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm_buf);
    return buf;
}

std::string nowMillis() {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto secs = clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &secs);
#else
    localtime_r(&secs, &tm_buf);
#endif
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                  static_cast<int>(ms.count()));
    return buf;
}

} // namespace

EventLogger::~EventLogger() { close(); }

bool EventLogger::open(const Config& cfg) {
    close();
    if (!cfg.logging_enabled || !cfg.logging_event_log_enabled) return false;

    std::error_code ec;
    std::filesystem::create_directories(cfg.logging_directory, ec);
    if (ec) {
        LOG_W("EventLogger: cannot create directory '%s' (%s) - disabling.",
              cfg.logging_directory.c_str(), ec.message().c_str());
        return false;
    }

    path_ = cfg.logging_directory + "/" + cfg.logging_experiment_name + "_" +
            fileStamp() + "_events.txt";

#if defined(_WIN32)
    std::FILE* f = nullptr;
    if (fopen_s(&f, path_.c_str(), "w") != 0) f = nullptr;
    file_ = f;
#else
    file_ = std::fopen(path_.c_str(), "w");
#endif
    if (!file_) {
        LOG_W("EventLogger: cannot open '%s' - disabling.", path_.c_str());
        path_.clear();
        return false;
    }
    warned_failure_ = false;
    LOG_I("EventLogger: opened '%s'.", path_.c_str());
    return true;
}

void EventLogger::close() {
    if (file_) {
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
        LOG_I("EventLogger: closed '%s'.", path_.c_str());
    }
    path_.clear();
}

void EventLogger::event(const std::string& message, bool also_console) {
    if (file_) {
        const std::string ts = nowMillis();
        int n = std::fprintf(file_, "[%s] %s\n", ts.c_str(), message.c_str());
        if (n < 0) {
            if (!warned_failure_) {
                LOG_W("EventLogger: write error - disabling further writes.");
                warned_failure_ = true;
            }
            close();
        } else {
            std::fflush(file_);
        }
    }
    if (also_console) {
        std::fprintf(stderr, "[EVT] %s\n", message.c_str());
    }
}

} // namespace dgd

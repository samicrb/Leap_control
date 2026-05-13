#include "util/Logger.hpp"

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <mutex>

namespace dgd {

static std::mutex g_log_mutex;

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

void Logger::configure(LogLevel level, const std::string& file_path) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    level_ = level;
    if (file_) {
        std::fclose(static_cast<std::FILE*>(file_));
        file_ = nullptr;
    }
    if (!file_path.empty()) {
#if defined(_WIN32)
        std::FILE* f = nullptr;
        if (fopen_s(&f, file_path.c_str(), "a") != 0) f = nullptr;
        file_ = f;
#else
        file_ = std::fopen(file_path.c_str(), "a");
#endif
        if (!file_) {
            std::fprintf(stderr, "[logger] cannot open log file '%s'\n", file_path.c_str());
        }
    }
}

void Logger::log(LogLevel lvl, const char* fmt, ...) {
    if (static_cast<int>(lvl) < static_cast<int>(level_)) return;

    std::lock_guard<std::mutex> lock(g_log_mutex);

    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    char ts[32];
    std::strftime(ts, sizeof(ts), "%H:%M:%S", &tm_buf);

    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    const char* lname = logLevelName(lvl);
    std::fprintf(stderr, "[%s] %-5s %s\n", ts, lname, msg);
    if (file_) {
        std::fprintf(static_cast<std::FILE*>(file_), "[%s] %-5s %s\n", ts, lname, msg);
        std::fflush(static_cast<std::FILE*>(file_));
    }
}

LogLevel parseLogLevel(const std::string& s) {
    if (s == "TRACE") return LogLevel::Trace;
    if (s == "DEBUG") return LogLevel::Debug;
    if (s == "INFO")  return LogLevel::Info;
    if (s == "WARN")  return LogLevel::Warn;
    if (s == "ERROR") return LogLevel::Error;
    return LogLevel::Info;
}

const char* logLevelName(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

} // namespace dgd

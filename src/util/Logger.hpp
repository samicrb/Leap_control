#pragma once

// Minimal thread-safe logger.
// - Single global instance (keeps usage simple in a demo).
// - Writes to stderr and optionally to a file.
// - No dependencies beyond <cstdio> / <mutex>.

#include <string>

namespace dgd {

enum class LogLevel { Trace = 0, Debug = 1, Info = 2, Warn = 3, Error = 4 };

class Logger {
public:
    static Logger& instance();

    void configure(LogLevel level, const std::string& file_path);
    void log(LogLevel level, const char* fmt, ...);

    LogLevel level() const { return level_; }

private:
    Logger() = default;
    LogLevel level_ = LogLevel::Info;
    void* file_ = nullptr; // FILE* (opaque to avoid <cstdio> in header)
};

LogLevel parseLogLevel(const std::string& s);
const char* logLevelName(LogLevel l);

} // namespace dgd

// Short macros. Compile-time cost is negligible for the demo loop rate.
#define LOG_T(...) ::dgd::Logger::instance().log(::dgd::LogLevel::Trace, __VA_ARGS__)
#define LOG_D(...) ::dgd::Logger::instance().log(::dgd::LogLevel::Debug, __VA_ARGS__)
#define LOG_I(...) ::dgd::Logger::instance().log(::dgd::LogLevel::Info,  __VA_ARGS__)
#define LOG_W(...) ::dgd::Logger::instance().log(::dgd::LogLevel::Warn,  __VA_ARGS__)
#define LOG_E(...) ::dgd::Logger::instance().log(::dgd::LogLevel::Error, __VA_ARGS__)

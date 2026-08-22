#pragma once
#include <atomic>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>

namespace tca {

enum class LogLevel : int { DEBUG = 0, INFO, WARNING, ERROR, FATAL };

class Logger {
public:
    static Logger& instance();

    void init(const std::string& logDir, LogLevel minLevel = LogLevel::INFO);

    void log(LogLevel level, std::string_view msg);

    // Convenience helpers
    void debug  (std::string_view m) { log(LogLevel::DEBUG,   m); }
    void info   (std::string_view m) { log(LogLevel::INFO,    m); }
    void warning(std::string_view m) { log(LogLevel::WARNING, m); }
    void error  (std::string_view m) { log(LogLevel::ERROR,   m); }
    void fatal  (std::string_view m) { log(LogLevel::FATAL,   m); }

    void flush();
    void shutdown();

private:
    Logger()  = default;
    ~Logger() = default;

    std::mutex    mtx_;
    std::ofstream file_;
    std::atomic<LogLevel> minLevel_{LogLevel::INFO};
    bool          initialized_{false};
    bool          echoConsole_{true};
};

// RAII stream builder ──────────────────────────────────────────────────────────
#define TCA_LOG(lvl, msg) \
    do { \
        std::ostringstream _ss; \
        _ss << msg; \
        ::tca::Logger::instance().log(::tca::LogLevel::lvl, _ss.str()); \
    } while(0)

#define LOG_DEBUG(m)   TCA_LOG(DEBUG,   m)
#define LOG_INFO(m)    TCA_LOG(INFO,    m)
#define LOG_WARN(m)    TCA_LOG(WARNING, m)
#define LOG_ERROR(m)   TCA_LOG(ERROR,   m)
#define LOG_FATAL(m)   TCA_LOG(FATAL,   m)

} // namespace tca

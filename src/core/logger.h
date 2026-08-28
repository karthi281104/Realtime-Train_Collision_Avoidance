#ifndef CORE_LOGGER_H
#define CORE_LOGGER_H

#include <string>
#include <fstream>
#include <sstream>
#include <chrono>

namespace core {

/**
 * @enum LogLevel
 * @brief Logging severity levels
 */
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

/**
 * @class Logger
 * @brief Centralized logging system for the simulation
 * 
 * WEEK 1 DELIVERABLE: Class declaration only
 * TODO: Implement in Week 2-3
 */
class Logger {
private:
    std::ofstream log_file;
    LogLevel current_level;
    bool console_output;
    
public:
    /**
     * @brief Initialize logger with file
     * @param filename Output log filename
     * @param level Minimum log level to record
     */
    void initialize(const std::string& filename, LogLevel level = LogLevel::INFO);
    
    /**
     * @brief Log a message
     * @param level Log level
     * @param message Message to log
     */
    void log(LogLevel level, const std::string& message);
    
    /**
     * @brief Log debug message
     */
    void debug(const std::string& message);
    
    /**
     * @brief Log info message
     */
    void info(const std::string& message);
    
    /**
     * @brief Log warning message
     */
    void warning(const std::string& message);
    
    /**
     * @brief Log error message
     */
    void error(const std::string& message);
    
    /**
     * @brief Log critical message
     */
    void critical(const std::string& message);
    
    /**
     * @brief Close logger
     */
    void close();
    
    /**
     * @brief Enable/disable console output
     */
    void setConsoleOutput(bool enabled);
};

} // namespace core

#endif // CORE_LOGGER_H

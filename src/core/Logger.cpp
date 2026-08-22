#include "core/Logger.hpp"
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace tca {

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

void Logger::init(const std::string& logDir, LogLevel minLevel) {
    std::lock_guard lk(mtx_);
    minLevel_ = minLevel;
    std::filesystem::create_directories(logDir);
    file_.open(logDir + "/system.log", std::ios::app);
    initialized_ = true;
    echoConsole_  = true;
}

void Logger::log(LogLevel level, std::string_view msg) {
    if(level < minLevel_) return;
    auto now   = std::chrono::system_clock::now();
    auto tt    = std::chrono::system_clock::to_time_t(now);
    auto ms    = std::chrono::duration_cast<std::chrono::milliseconds>(
                     now.time_since_epoch()) % 1000;

    const char* lvlStr[] = {"[DBG]","[INF]","[WRN]","[ERR]","[FAT]"};
    std::ostringstream ss;
    {
        std::tm t{};
#ifdef _WIN32
        localtime_s(&t, &tt);
#else
        localtime_r(&tt, &t);
#endif
        ss << std::put_time(&t, "%H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << ms.count()
           << ' ' << lvlStr[static_cast<int>(level)]
           << ' ' << msg << '\n';
    }
    std::lock_guard lk(mtx_);
    if(initialized_) file_ << ss.str();
    if(echoConsole_) std::cout << ss.str() << std::flush;
}

void Logger::flush() {
    std::lock_guard lk(mtx_);
    if(initialized_) file_.flush();
}

void Logger::shutdown() {
    std::lock_guard lk(mtx_);
    if(initialized_) { file_.flush(); file_.close(); initialized_ = false; }
}

} // namespace tca

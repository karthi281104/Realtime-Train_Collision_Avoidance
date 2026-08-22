#pragma once
#include "core/Types.hpp"
#include "train/TrainTypes.hpp"
#include <fstream>
#include <mutex>
#include <string>

namespace tca {

// Forward declarations to break circular dependency
struct ConflictInfo;
struct Resolution;

class EventLogger {
public:
    void init(const std::string& logDir);
    void logTrainState(const TrainData& td, double simNow);
    void logConflict  (const ConflictInfo& ci);
    void logResolution(const Resolution& res, double simNow);
    void logSystem    (const std::string& msg, double simNow);
    void flush();

private:
    void write(const std::string& filename, const std::string& line);

    std::mutex   mtx_;
    std::string  logDir_;
    bool         initialized_{false};
};

} // namespace tca

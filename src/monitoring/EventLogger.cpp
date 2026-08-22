#include "monitoring/EventLogger.hpp"
#include "prediction/CollisionPredictor.hpp"
#include "safety/ConflictResolver.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>


namespace tca {

void EventLogger::init(const std::string& logDir) {
    std::lock_guard lk(mtx_);
    std::filesystem::create_directories(logDir);
    logDir_      = logDir;
    initialized_ = true;

    // Write CSV headers
    write("train_state.csv",
          "sim_time,train_id,name,type,position_m,velocity_kmh,accel_ms2,state,track,sensor");
    write("conflicts.csv",
          "sim_time,conflict_id,type,train_a,train_b,ttc_s,sep_m,req_sep_m,risk,action");
    write("control_actions.csv",
          "sim_time,conflict_id,train_id,action,target_speed_kmh,applied");
    write("system.log", "=== Train Collision Avoidance System Log ===");
}

void EventLogger::logTrainState(const TrainData& td, double simNow) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << simNow << ","
       << td.id << ","
       << td.name << ","
       << toString(td.type) << ","
       << static_cast<int>(td.positionM) << ","
       << std::setprecision(1) << td.velocityMs * kMsToKmh << ","
       << std::setprecision(2) << td.accelerationMs2 << ","
       << toString(td.state) << ","
       << td.currentTrackId << ","
       << (td.posSensor == SensorStatus::NORMAL ? "OK" : "DEGRADED");
    write("train_state.csv", ss.str());
}

void EventLogger::logConflict(const ConflictInfo& ci) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << ci.detectedAtTime << ","
       << ci.id << ","
       << toString(ci.type) << ","
       << ci.trainA << "," << ci.trainB << ","
       << std::setprecision(1) << ci.ttcSeconds << ","
       << static_cast<int>(ci.separationM) << ","
       << static_cast<int>(ci.requiredSepM) << ","
       << toString(ci.risk) << ","
       << toString(ci.recommendedAction);
    write("conflicts.csv", ss.str());
}

void EventLogger::logResolution(const Resolution& res, double simNow) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << simNow << ","
       << res.conflictId << ","
       << res.targetTrain << ","
       << toString(res.action) << ","
       << std::setprecision(1) << res.newTargetSpeedMs * kMsToKmh << ","
       << (res.applied ? "YES" : "NO");
    write("control_actions.csv", ss.str());
}

void EventLogger::logSystem(const std::string& msg, double simNow) {
    std::ostringstream ss;
    ss << "[t=" << std::fixed << std::setprecision(2) << simNow << "s] " << msg;
    write("system.log", ss.str());
}

void EventLogger::flush() {}

void EventLogger::write(const std::string& filename, const std::string& line) {
    if(!initialized_) return;
    std::ofstream f(logDir_ + "/" + filename, std::ios::app);
    if(f) f << line << "\n";
}

} // namespace tca

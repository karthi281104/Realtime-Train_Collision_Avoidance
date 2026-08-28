#ifndef CORE_CONFIG_H
#define CORE_CONFIG_H

#include <chrono>
#include <string>

namespace core {

/**
 * @class Config
 * @brief Central configuration for the train collision avoidance system
 * 
 * Week 1 Deliverable: Configuration declarations only
 */
class Config {
public:
    // Simulation parameters
    static constexpr double SIMULATION_TIMESTEP_MS = 50.0;      // 50ms fixed timestep
    static constexpr double SIMULATION_MAX_DURATION_S = 3600.0; // 1 hour max
    
    // Physics parameters
    static constexpr double TRAIN_MAX_SPEED_MS = 50.0;          // 50 m/s (180 km/h)
    static constexpr double TRAIN_MAX_ACCEL_MS2 = 1.0;          // 1 m/s² acceleration
    static constexpr double TRAIN_MAX_DECEL_MS2 = 2.5;          // 2.5 m/s² braking
    static constexpr double TRAIN_EMERGENCY_DECEL_MS2 = 4.0;    // 4.0 m/s² emergency
    
    // Safety thresholds
    static constexpr double COLLISION_WARNING_DISTANCE_M = 500.0;
    static constexpr double CRITICAL_DISTANCE_M = 100.0;
    static constexpr double EMERGENCY_DISTANCE_M = 20.0;
    
    // Communication parameters
    static constexpr double COMM_DELAY_MS = 100.0;
    static constexpr double PACKET_LOSS_RATE = 0.05;
    
    // Graph/Network parameters
    static constexpr int MAX_TRACKS = 1000;
    static constexpr int MAX_STATIONS = 500;
    static constexpr int MAX_TRAINS = 100;
};

} // namespace core

#endif // CORE_CONFIG_H

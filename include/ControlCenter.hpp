#pragma once
#include "simulation/SimulationEngine.hpp"
#include "safety/ConflictResolver.hpp"
#include "communication/MessageBus.hpp"
#include "monitoring/Dashboard.hpp"
#include "monitoring/EventLogger.hpp"
#include "monitoring/ScenarioManager.hpp"
#include "core/Config.hpp"
#include <cstddef>
#include <memory>

namespace tca {

/// Top-level system integrator.
class ControlCenter {
public:
    ControlCenter();
    ~ControlCenter();

    bool init(const std::string& configPath, const std::string& scenario,
              std::size_t trainCount = 0);

    void run();
    void stop();

    // For testing: run a specific scenario synchronously for N seconds.
    void runScenario(const std::string& name, double seconds, std::size_t trainCount = 0);

private:
    Config            cfg_;
    RailwayNetwork    net_;
    RouteManager      rm_;
    TrainStateManager tsm_;
    ConflictManager   cm_;
    ConflictResolver  cr_;
    EventLogger       el_;
    SimulationEngine  sim_;   // owns its own SimulationClock
    Dashboard         dash_;  // initialized AFTER sim_
    MessageBus        bus_;
    ScenarioManager   scenarios_;
};

} // namespace tca

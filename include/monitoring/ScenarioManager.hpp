#pragma once
#include "train/TrainStateManager.hpp"
#include "railway/RailwayNetwork.hpp"
#include "railway/RouteManager.hpp"
#include <functional>
#include <string>

namespace tca {

struct ScenarioConfig {
    std::string name;
    std::string description;
    double      durationSeconds{60.0};
};

/// Encapsulates a pre-built scenario for deterministic testing.
class ScenarioManager {
public:
    ScenarioManager(TrainStateManager& tsm,
                    RailwayNetwork&    net,
                    RouteManager&      rm);

    /// Load one of the pre-defined scenarios by name.
    ScenarioConfig load(const std::string& scenarioName);

    // Pre-defined scenarios
    ScenarioConfig scenario_NormalOps();
    ScenarioConfig scenario_RearEnd();
    ScenarioConfig scenario_HeadOn();
    ScenarioConfig scenario_Junction();
    ScenarioConfig scenario_CommDelay();
    ScenarioConfig scenario_PacketLoss();
    ScenarioConfig scenario_SensorFault();
    ScenarioConfig scenario_MultiConflict();
    ScenarioConfig scenario_EmergencyBrake();
    ScenarioConfig scenario_HighDensity();

private:
    void clearAll();
    void buildLinearNetwork(int stationCount, double segmentLenM);
    void buildBranchNetwork();

    TrainStateManager& tsm_;
    RailwayNetwork&    net_;
    RouteManager&      rm_;
};

} // namespace tca

#pragma once
#include "train/TrainStateManager.hpp"
#include "railway/RailwayNetwork.hpp"
#include "railway/RouteManager.hpp"
#include "core/SimulationClock.hpp"

namespace tca {

/// Advances all train positions by one fixed time-step.
class MovementEngine {
public:
    MovementEngine(TrainStateManager& tsm,
                   RailwayNetwork&    net,
                   RouteManager&      rm,
                   SimulationClock&   clk);

    /// Execute one tick for all trains.
    void tick();

private:
    void updateTrain(Train& t);
    void handleTrackTransition(Train& t);
    void enforceSpeedLimit(Train& t);
    void enforceTargetSpeed(Train& t);

    TrainStateManager& tsm_;
    RailwayNetwork&    net_;
    RouteManager&      rm_;
    SimulationClock&   clk_;
};

} // namespace tca

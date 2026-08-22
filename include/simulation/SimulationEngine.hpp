#pragma once
#include "simulation/MovementEngine.hpp"
#include "safety/ConflictManager.hpp"
#include "safety/ConflictResolver.hpp"
#include "monitoring/EventLogger.hpp"
#include <atomic>
#include <thread>


namespace tca {

class SimulationEngine {
public:
    SimulationEngine(TrainStateManager& tsm,
                     RailwayNetwork&    net,
                     RouteManager&      rm,
                     ConflictManager&   cm,
                     ConflictResolver&  cr,
                     EventLogger&       el);

    void start();
    void stop();
    void runBlocking(double durationSeconds);  // for sequential testing

    bool isRunning() const { return running_.load(); }
    double simTime() const { return clk_.simTime(); }

    SimulationClock& clock() { return clk_; }

private:
    void loop();

    TrainStateManager& tsm_;
    RailwayNetwork&    net_;
    RouteManager&      rm_;
    ConflictManager&   cm_;
    ConflictResolver&  cr_;
    EventLogger&       el_;
    SimulationClock    clk_;
    MovementEngine     move_;

    std::atomic<bool>  running_{false};
    std::thread        thread_;
};

} // namespace tca

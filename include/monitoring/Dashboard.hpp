#pragma once
#include "safety/ConflictManager.hpp"
#include "train/TrainStateManager.hpp"
#include "railway/RailwayNetwork.hpp"
#include "core/SimulationClock.hpp"
#include <atomic>
#include <thread>

namespace tca {

/// Terminal dashboard running at ~10 Hz.
class Dashboard {
public:
    Dashboard(TrainStateManager& tsm,
              ConflictManager&   cm,
              RailwayNetwork&    net,
              SimulationClock&   clk);

    void start();
    void stop();

private:
    void loop();
    void render();
    void clearScreen();
    void printHeader  ();
    void printTrains  ();
    void printConflicts();
    void printFooter  ();

    TrainStateManager& tsm_;
    ConflictManager&   cm_;
    RailwayNetwork&    net_;
    SimulationClock&   clk_;

    std::atomic<bool> running_{false};
    std::thread       thread_;
};

} // namespace tca

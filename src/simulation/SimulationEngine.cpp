#include "simulation/SimulationEngine.hpp"
#include "core/Logger.hpp"
#include <sstream>

namespace tca {

SimulationEngine::SimulationEngine(TrainStateManager& tsm,
                                   RailwayNetwork&    net,
                                   RouteManager&      rm,
                                   ConflictManager&   cm,
                                   ConflictResolver&  cr,
                                   EventLogger&       el)
    : tsm_(tsm), net_(net), rm_(rm), cm_(cm), cr_(cr), el_(el),
      clk_(kDefaultDt),
      move_(tsm_, net_, rm_, clk_)
{}

void SimulationEngine::start() {
    running_.store(true);
    thread_ = std::thread(&SimulationEngine::loop, this);
    LOG_INFO("SimulationEngine started (dt=" << kDefaultDt*1000 << "ms)");
}

void SimulationEngine::stop() {
    running_.store(false);
    if(thread_.joinable()) thread_.join();
    LOG_INFO("SimulationEngine stopped at t=" << clk_.simTime() << "s");
}

void SimulationEngine::runBlocking(double durationSeconds) {
    LOG_INFO("Running simulation for " << durationSeconds << "s...");
    clk_.reset();
    while(clk_.simTime() < durationSeconds) {
        // Movement
        move_.tick();

        // Prediction + conflict detection
        auto snap      = tsm_.snapshot();
        auto conflicts = cm_.update(snap, clk_.simTime());

        // Resolution
        if(!conflicts.empty()) {
            auto resolutions = cr_.resolve(conflicts, tsm_, net_, clk_.simTime());
            for(auto& r : resolutions) el_.logResolution(r, clk_.simTime());
        }

        // Log post-resolution train states
        auto postSnap = tsm_.snapshot();
        for(auto& td : postSnap) el_.logTrainState(td, clk_.simTime());

        clk_.tick();
    }
    LOG_INFO("Simulation complete. " << clk_.tickCount() << " ticks.");
}

void SimulationEngine::loop() {
    clk_.reset();
    while(running_.load()) {
        move_.tick();

        auto snap      = tsm_.snapshot();
        auto conflicts = cm_.update(snap, clk_.simTime());
        for(const auto& conflict : conflicts)
            el_.logConflict(conflict);

        if(!conflicts.empty()) {
            auto resolutions = cr_.resolve(conflicts, tsm_, net_, clk_.simTime());
            for(const auto& resolution : resolutions)
                el_.logResolution(resolution, clk_.simTime());
        }

        auto postSnap = tsm_.snapshot();
        for(const auto& td : postSnap)
            el_.logTrainState(td, clk_.simTime());

        clk_.tick();
    }
}

} // namespace tca

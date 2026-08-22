#include "simulation/MovementEngine.hpp"
#include "train/TrainPhysics.hpp"
#include "core/Logger.hpp"
#include <algorithm>
#include <sstream>

namespace tca {

MovementEngine::MovementEngine(TrainStateManager& tsm,
                               RailwayNetwork&    net,
                               RouteManager&      rm,
                               SimulationClock&   clk)
    : tsm_(tsm), net_(net), rm_(rm), clk_(clk) {}

void MovementEngine::tick() {
    tsm_.forEach([this](Train& t){ updateTrain(t); });
}

void MovementEngine::updateTrain(Train& t) {
    auto& d = t.data();
    if(d.state == TrainState::IDLE || d.state == TrainState::STOPPED) return;

    // 1. Compute acceleration toward target
    d.accelerationMs2 = physics::computeAcceleration(d);

    // 2. Enforce track speed limit
    enforceSpeedLimit(t);

    // 3. Euler integration
    t.applyPhysics(clk_.dt());

    // 4. Timestamp
    d.simTimestamp = clk_.simTime();

    // 5. Handle track boundary / route transitions
    handleTrackTransition(t);
}

void MovementEngine::enforceSpeedLimit(Train& t) {
    auto& d = t.data();
    if(auto* tk = net_.track(d.currentTrackId)) {
        double limit = tk->speedLimitMs;
        if(d.targetSpeedMs > limit) d.targetSpeedMs = limit;
        if(d.velocityMs > limit + kEpsilon) {
            // Force braking to limit
            d.accelerationMs2 = -d.spec.normalBrakeMs2;
        }
    }
}

void MovementEngine::handleTrackTransition(Train& t) {
    auto& d = t.data();
    auto* tk = net_.track(d.currentTrackId);
    if(!tk) return;

    if(d.positionM >= tk->lengthM) {
        // Advance route segment
        auto* route = rm_.routeForTrain(d.id);
        if(route && !route->isComplete()) {
            // Leave current track
            net_.trainLeavesTrack(d.id, d.currentTrackId);

            double overflow = d.positionM - tk->lengthM;
            route->advanceSegment();

            TrackId nextTk = route->currentTrack();
            if(nextTk == 0) {
                // Reached destination
                d.velocityMs = 0.0;
                d.accelerationMs2 = 0.0;
                d.state = TrainState::STOPPED;
                LOG_INFO(d.name << " reached destination.");
                return;
            }

            d.currentTrackId = nextTk;
            d.positionM = overflow;

            auto* newTk = net_.track(nextTk);
            if(newTk) {
                d.currentFromNode = newTk->fromNode;
                d.currentToNode   = newTk->toNode;
                net_.trainEntersTrack(d.id, nextTk);
            }
        } else {
            // No more route → stop at end of track
            d.positionM = tk->lengthM;
            d.velocityMs = 0.0;
            d.state = TrainState::STOPPED;
        }
    }
}

} // namespace tca

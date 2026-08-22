#pragma once
#include "safety/ConflictManager.hpp"
#include "train/TrainStateManager.hpp"

namespace tca {

struct Resolution {
    ConflictId    conflictId{0};
    TrainId       targetTrain{0};
    ConflictAction action{ConflictAction::NONE};
    double         newTargetSpeedMs{-1.0};   // -1 = no change
    bool           applied{false};
};

class ConflictResolver {
public:
    /// Try actions in safety hierarchy, pick least disruptive that works.
    std::vector<Resolution>
    resolve(const std::vector<ConflictInfo>& conflicts,
            TrainStateManager& tsm,
            const RailwayNetwork& net,
            double simNow);

private:
    Resolution resolveOne(const ConflictInfo& ci,
                          TrainStateManager& tsm,
                          const RailwayNetwork& net,
                          double simNow);

    bool simulateResolution(const ConflictInfo& ci,
                            const TrainData& modified,
                            const TrainData& other,
                            const RailwayNetwork& net) const;

    void applyResolution(const Resolution& res, TrainStateManager& tsm);
};

} // namespace tca

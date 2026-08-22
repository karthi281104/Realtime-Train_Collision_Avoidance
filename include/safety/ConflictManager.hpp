#pragma once
#include "prediction/CollisionPredictor.hpp"
#include "train/TrainStateManager.hpp"
#include <mutex>
#include <vector>
#include <unordered_map>

namespace tca {

class RiskAssessor {
public:
    RiskLevel assess(const PredictionResult& pr) const;

    double ttcThreshold  (RiskLevel rl) const;
    double sepThreshold  (RiskLevel rl) const;
};

class ConflictManager {
public:
    explicit ConflictManager(const RailwayNetwork& net);

    /// Called each sim tick with fresh snapshot.
    /// Returns list of active conflicts (may be empty).
    std::vector<ConflictInfo> update(const std::vector<TrainData>& snapshot,
                                     double simNow);

    std::vector<ConflictInfo> activeConflicts() const;

    void clearResolved();

private:
    CollisionPredictor           predictor_;
    RiskAssessor                 assessor_;
    std::vector<ConflictInfo>    active_;
    ConflictId                   nextId_{1};
    mutable std::mutex           mtx_;
};

} // namespace tca

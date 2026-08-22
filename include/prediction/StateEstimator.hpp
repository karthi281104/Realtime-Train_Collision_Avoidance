#pragma once
#include "train/TrainTypes.hpp"

namespace tca {

/// Dead-reckoning estimator: given last known state + elapsed time
/// produces an estimated state with uncertainty.
struct EstimatedState {
    TrainData data;
    double    positionUncertaintyM{0.0};  // ±metres
    double    velocityUncertaintyMs{0.0};
    bool      isEstimated{false};
    double    estimatedAtTime{0.0};
};

class StateEstimator {
public:
    /// Project train state forward by elapsedS seconds using motion model.
    EstimatedState estimate(const TrainData& last, double elapsedS) const;

    /// Merge estimated state into train (when real update is missing).
    void applyEstimate(TrainData& td, const EstimatedState& est) const;
};

} // namespace tca

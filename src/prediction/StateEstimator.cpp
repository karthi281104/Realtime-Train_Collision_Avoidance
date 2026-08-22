#include "prediction/StateEstimator.hpp"
#include "train/TrainPhysics.hpp"
#include <cmath>

namespace tca {

EstimatedState StateEstimator::estimate(const TrainData& last, double elapsedS) const {
    EstimatedState est;
    est.data = last;
    est.isEstimated = true;
    est.estimatedAtTime = last.simTimestamp + elapsedS;

    // Dead-reckoning: constant velocity + deceleration model
    double dt = elapsedS;
    double accel = physics::computeAcceleration(last);
    physics::integrate(est.data.positionM, est.data.velocityMs, accel, dt);

    // Uncertainty grows with time: σ_pos = 0.5 * max_accel * t²
    double maxAccel = last.spec.normalBrakeMs2;
    est.positionUncertaintyM = 0.5 * maxAccel * elapsedS * elapsedS;
    est.velocityUncertaintyMs= maxAccel * elapsedS;

    est.data.posSensor   = SensorStatus::DEGRADED;
    est.data.speedSensor = SensorStatus::DEGRADED;
    est.data.simTimestamp = est.estimatedAtTime;

    return est;
}

void StateEstimator::applyEstimate(TrainData& td, const EstimatedState& est) const {
    td.positionM  = est.data.positionM;
    td.velocityMs = est.data.velocityMs;
    td.posSensor  = SensorStatus::DEGRADED;
    td.simTimestamp = est.estimatedAtTime;
}

} // namespace tca

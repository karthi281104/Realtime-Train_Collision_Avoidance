#pragma once
#include "train/TrainTypes.hpp"

namespace tca {

/// Pure functions: no side-effects, easily unit-tested.
namespace physics {

/// Euler step: x_new = x + v*dt + 0.5*a*dt²,  v_new = v + a*dt
void integrate(double& pos, double& vel, double accel, double dt);

/// Choose acceleration to drive toward targetSpeed (PID-lite).
double computeAcceleration(const TrainData& td);

/// Braking distance = v²/(2·a) + v·reactionTime
double brakingDistance(double speedMs, double decelMs2, double reactionTimeS);

/// Required separation = trainLength + brakingDist + safetyMargin
double requiredSeparation(const TrainData& td, bool emergency = false);

/// TTC for two trains on same track, same direction (trailing behind leading).
/// Returns kInfinity if converging impossible / already safe.
double ttcSameDirection(double posLeading,   double velLeading,
                        double posTrailing,  double velTrailing,
                        double safeGapM);

/// TTC head-on: both approaching each other.
double ttcHeadOn(double posA, double velA, double posB, double velB);

} // namespace physics
} // namespace tca

#include "train/TrainPhysics.hpp"
#include <algorithm>
#include <cmath>

namespace tca::physics {

void integrate(double& pos, double& vel, double accel, double dt) {
    // x_{t+dt} = x_t + v_t*dt + 0.5*a*dt²
    pos += vel * dt + 0.5 * accel * dt * dt;
    // v_{t+dt} = v_t + a*dt
    vel += accel * dt;
    // Clamp: speed cannot be negative (trains don't reverse here)
    if(vel < 0.0) vel = 0.0;
}

double computeAcceleration(const TrainData& td) {
    double diff = td.targetSpeedMs - td.velocityMs;
    if(td.emergencyBrakeActive)
        return -td.spec.emergencyBrakeMs2;
    if(diff > kEpsilon)
        return std::min(diff / 1.0, td.spec.normalAccelMs2);  // 1-second ramp
    if(diff < -kEpsilon)
        return std::max(diff / 1.0, -td.spec.normalBrakeMs2);
    return 0.0;
}

double brakingDistance(double speedMs, double decelMs2, double reactionTimeS) {
    // reaction: d_r = v * t_r
    // braking:  d_b = v² / (2·a)
    return speedMs * reactionTimeS + (speedMs * speedMs) / (2.0 * decelMs2);
}

double requiredSeparation(const TrainData& td, bool emergency) {
    double decel = emergency ? td.spec.emergencyBrakeMs2 : td.spec.normalBrakeMs2;
    return td.spec.lengthM
         + brakingDistance(td.velocityMs, decel, td.spec.reactionTimeS)
         + td.spec.safetyMarginM;
}

double ttcSameDirection(double posLeading,  double velLeading,
                        double posTrailing, double velTrailing,
                        double safeGapM) {
    // Trailing is behind Leading (posTrailing < posLeading)
    double gap      = posLeading - posTrailing;
    double relVel   = velTrailing - velLeading; // positive → closing
    if(relVel <= kEpsilon) return kInfinity;    // diverging or same speed
    double timeToGap = (gap - safeGapM) / relVel;
    return timeToGap > 0 ? timeToGap : 0.0;
}

double ttcHeadOn(double posA, double velA, double posB, double velB) {
    // A and B approaching (posA < posB, velA > 0 forward, velB > 0 forward on reverse direction)
    double gap    = posB - posA;
    double relVel = velA + velB;  // combined closing speed
    if(relVel <= kEpsilon) return kInfinity;
    return gap / relVel;
}

} // namespace tca::physics

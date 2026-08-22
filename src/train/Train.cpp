#include "train/TrainTypes.hpp"
#include "train/TrainPhysics.hpp"
#include <cmath>
#include <string_view>

namespace tca {

std::string_view toString(TrainType t) {
    switch(t) {
        case TrainType::PASSENGER: return "PASSENGER";
        case TrainType::EXPRESS:   return "EXPRESS";
        case TrainType::FREIGHT:   return "FREIGHT";
    }
    return "UNKNOWN";
}

TrainSpec TrainSpec::forPassenger() {
    TrainSpec s;
    s.maxSpeedMs        = 160.0 * kMsToMs;   // 160 km/h
    s.normalAccelMs2    = 0.8;
    s.normalBrakeMs2    = 1.1;
    s.emergencyBrakeMs2 = 2.5;
    s.lengthM           = 180.0;
    s.reactionTimeS     = 1.5;
    s.safetyMarginM     = 50.0;
    return s;
}

TrainSpec TrainSpec::forExpress() {
    TrainSpec s;
    s.maxSpeedMs        = 250.0 * kMsToMs;   // 250 km/h
    s.normalAccelMs2    = 1.0;
    s.normalBrakeMs2    = 1.3;
    s.emergencyBrakeMs2 = 3.0;
    s.lengthM           = 200.0;
    s.reactionTimeS     = 1.2;
    s.safetyMarginM     = 80.0;
    return s;
}

TrainSpec TrainSpec::forFreight() {
    TrainSpec s;
    s.maxSpeedMs        = 100.0 * kMsToMs;   // 100 km/h
    s.normalAccelMs2    = 0.4;
    s.normalBrakeMs2    = 0.6;
    s.emergencyBrakeMs2 = 1.5;
    s.lengthM           = 500.0;
    s.reactionTimeS     = 2.5;
    s.safetyMarginM     = 100.0;
    return s;
}

// ─── Train base ───────────────────────────────────────────────────────────────

void Train::applyPhysics(double dt) {
    physics::integrate(data_.positionM, data_.velocityMs,
                       data_.accelerationMs2, dt);
    if(data_.velocityMs <= 0.0) {
        data_.velocityMs = 0.0;
        if(data_.state == TrainState::BRAKING ||
           data_.state == TrainState::EMERGENCY_BRAKING)
            data_.state = TrainState::STOPPED;
    }
}

double Train::brakingDistance(bool emergency) const {
    return physics::brakingDistance(
        data_.velocityMs,
        emergency ? data_.spec.emergencyBrakeMs2 : data_.spec.normalBrakeMs2,
        data_.spec.reactionTimeS);
}

double Train::requiredSeparation() const {
    return physics::requiredSeparation(data_);
}

} // namespace tca

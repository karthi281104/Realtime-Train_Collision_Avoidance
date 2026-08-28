#include "train.h"
#include <cmath>
#include <stdexcept>

namespace train {

Train::Train(int id, const std::string& name, TrainType train_type)
    : train_id(id), train_name(name), type(train_type), state(TrainState::IDLE),
      mass_kg(50000.0), length_meters(100.0), max_speed_ms(50.0),
      max_acceleration_ms2(1.0), max_braking_decel_ms2(2.5),
      emergency_decel_ms2(4.0), reaction_time_s(2.0),
      current_node_id(-1), next_node_id(-1), distance_to_next_m(0.0),
      time_since_last_brake_s(0.0), is_faulty(false), fault_description("") {
    initializeProperties();
}

void Train::initializeProperties() {
    switch (type) {
        case TrainType::PASSENGER:
            mass_kg = 50000.0;
            max_speed_ms = 50.0;          // 180 km/h
            max_acceleration_ms2 = 1.0;
            max_braking_decel_ms2 = 2.5;
            emergency_decel_ms2 = 4.0;
            break;
        case TrainType::FREIGHT:
            mass_kg = 100000.0;
            max_speed_ms = 35.0;          // 126 km/h
            max_acceleration_ms2 = 0.5;
            max_braking_decel_ms2 = 1.8;
            emergency_decel_ms2 = 2.5;
            break;
        case TrainType::EXPRESS:
            mass_kg = 45000.0;
            max_speed_ms = 65.0;          // 234 km/h
            max_acceleration_ms2 = 1.5;
            max_braking_decel_ms2 = 3.0;
            emergency_decel_ms2 = 4.5;
            break;
        case TrainType::LOCAL:
            mass_kg = 40000.0;
            max_speed_ms = 40.0;          // 144 km/h
            max_acceleration_ms2 = 0.8;
            max_braking_decel_ms2 = 2.2;
            emergency_decel_ms2 = 3.5;
            break;
    }
}

double Train::calculateBrakingDistance(double current_velocity, double target_velocity, double deceleration) const {
    if (deceleration <= 0.0 || current_velocity <= target_velocity) {
        return 0.0;
    }
    
    // d = (v^2 - u^2) / (2*a)
    double v_squared = current_velocity * current_velocity;
    double u_squared = target_velocity * target_velocity;
    return (v_squared - u_squared) / (2.0 * deceleration);
}

void Train::updateState() {
    if (is_faulty) {
        state = TrainState::FAULT;
        return;
    }
    
    if (physics.velocity_ms < 0.1) {
        state = TrainState::STOPPED;
    } else if (physics.acceleration_ms2 > 0.1) {
        state = TrainState::ACCELERATING;
    } else if (physics.acceleration_ms2 < -0.1) {
        state = TrainState::DECELERATING;
    } else {
        state = TrainState::CRUISING;
    }
}

void Train::setPosition(double position) {
    physics.position_meters = position;
}

void Train::setTargetVelocity(double velocity) {
    physics.target_velocity_ms = std::min(velocity, max_speed_ms);
}

void Train::setRoute(const std::vector<int>& path) {
    route_path = path;
}

void Train::setCurrentNode(int current, int next) {
    current_node_id = current;
    next_node_id = next;
}

void Train::updatePhysics(double delta_time) {
    if (is_faulty || state == TrainState::FAULT) {
        physics.acceleration_ms2 = 0.0;
        physics.velocity_ms = std::max(0.0, physics.velocity_ms - emergency_decel_ms2 * delta_time);
        updateState();
        return;
    }
    
    // Calculate desired acceleration to reach target velocity
    double velocity_diff = physics.target_velocity_ms - physics.velocity_ms;
    double desired_accel = 0.0;
    
    if (velocity_diff > 0.1) {
        // Need to accelerate
        desired_accel = std::min(velocity_diff / delta_time, max_acceleration_ms2);
    } else if (velocity_diff < -0.1) {
        // Need to decelerate
        desired_accel = std::max(velocity_diff / delta_time, -max_braking_decel_ms2);
    }
    
    physics.acceleration_ms2 = desired_accel;
    
    // Update velocity using Euler method: v = v0 + a*dt
    physics.velocity_ms += physics.acceleration_ms2 * delta_time;
    physics.velocity_ms = std::max(0.0, physics.velocity_ms);
    physics.velocity_ms = std::min(physics.velocity_ms, max_speed_ms);
    
    // Update position: s = s0 + v*dt
    physics.position_meters += physics.velocity_ms * delta_time;
    
    // Update distance to next node
    if (distance_to_next_m > 0.0) {
        distance_to_next_m -= physics.velocity_ms * delta_time;
        distance_to_next_m = std::max(0.0, distance_to_next_m);
    }
    
    updateState();
}

void Train::applyBrake(double braking_level) {
    if (braking_level < 0.0 || braking_level > 1.0) {
        throw std::invalid_argument("Braking level must be between 0.0 and 1.0");
    }
    
    double braking_decel = max_braking_decel_ms2 * braking_level;
    physics.target_velocity_ms = std::max(0.0, physics.velocity_ms - braking_decel * 0.05); // 50ms timestep
    time_since_last_brake_s = 0.0;
}

void Train::emergencyBrake() {
    physics.target_velocity_ms = 0.0;
    state = TrainState::EMERGENCY_BRAKE;
    time_since_last_brake_s = 0.0;
}

void Train::applyAcceleration(double acceleration_level) {
    if (acceleration_level < 0.0 || acceleration_level > 1.0) {
        throw std::invalid_argument("Acceleration level must be between 0.0 and 1.0");
    }
    
    double accel = max_acceleration_ms2 * acceleration_level;
    physics.target_velocity_ms = std::min(max_speed_ms, physics.velocity_ms + accel * 0.05); // 50ms timestep
}

void Train::hold() {
    physics.target_velocity_ms = 0.0;
    state = TrainState::IDLE;
}

void Train::setFault(bool faulty, const std::string& description) {
    is_faulty = faulty;
    fault_description = description;
    if (faulty) {
        state = TrainState::FAULT;
    }
}

void Train::clearFault() {
    is_faulty = false;
    fault_description = "";
    updateState();
}

std::string Train::getStateString() const {
    switch (state) {
        case TrainState::IDLE:            return "IDLE";
        case TrainState::ACCELERATING:    return "ACCELERATING";
        case TrainState::CRUISING:        return "CRUISING";
        case TrainState::DECELERATING:    return "DECELERATING";
        case TrainState::EMERGENCY_BRAKE: return "EMERGENCY_BRAKE";
        case TrainState::STOPPED:         return "STOPPED";
        case TrainState::FAULT:           return "FAULT";
        default:                          return "UNKNOWN";
    }
}

std::string Train::getTypeString() const {
    switch (type) {
        case TrainType::PASSENGER: return "PASSENGER";
        case TrainType::FREIGHT:   return "FREIGHT";
        case TrainType::EXPRESS:   return "EXPRESS";
        case TrainType::LOCAL:     return "LOCAL";
        default:                   return "UNKNOWN";
    }
}

void Train::reset() {
    physics.position_meters = 0.0;
    physics.velocity_ms = 0.0;
    physics.acceleration_ms2 = 0.0;
    physics.target_velocity_ms = 0.0;
    state = TrainState::IDLE;
    is_faulty = false;
    fault_description = "";
    current_node_id = -1;
    next_node_id = -1;
    distance_to_next_m = 0.0;
    time_since_last_brake_s = 0.0;
}

} // namespace train

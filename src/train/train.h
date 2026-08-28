#ifndef TRAIN_TRAIN_H
#define TRAIN_TRAIN_H

#include <string>
#include <vector>
#include <cmath>
#include "../core/config.h"

namespace train {

/**
 * @enum TrainType
 * @brief Classification of train types
 */
enum class TrainType {
    PASSENGER,
    FREIGHT,
    EXPRESS,
    LOCAL
};

/**
 * @enum TrainState
 * @brief Current operational state of the train
 */
enum class TrainState {
    IDLE,           // Stationary at station
    ACCELERATING,   // Speeding up
    CRUISING,       // Maintaining constant speed
    DECELERATING,   // Slowing down
    EMERGENCY_BRAKE, // Emergency braking
    STOPPED,        // Complete stop
    FAULT           // System fault
};

/**
 * @struct PhysicsState
 * @brief Real-time physics data of the train
 */
struct PhysicsState {
    double position_meters;          // Position on track (cumulative)
    double velocity_ms;              // Current velocity (m/s)
    double acceleration_ms2;         // Current acceleration (m/s²)
    double target_velocity_ms;       // Desired velocity (m/s)
    
    PhysicsState()
        : position_meters(0.0), velocity_ms(0.0), acceleration_ms2(0.0), target_velocity_ms(0.0) {}
};

/**
 * @class Train
 * @brief Represents a train with physics simulation and state management
 * 
 * WEEK 1 DELIVERABLE: Fully Implemented
 * - Train physics simulation (Euler integration)
 * - State management
 * - Speed control and braking
 * - Real-time kinematics
 */
class Train {
private:
    int train_id;
    std::string train_name;
    TrainType type;
    TrainState state;
    
    // Physical properties
    double mass_kg;                  // Train mass in kg
    double length_meters;            // Train length
    double max_speed_ms;             // Maximum operational speed
    double max_acceleration_ms2;     // Maximum acceleration
    double max_braking_decel_ms2;    // Maximum braking deceleration
    double emergency_decel_ms2;      // Emergency braking deceleration
    double reaction_time_s;          // Driver/system reaction time
    
    // Current physics state
    PhysicsState physics;
    
    // Route information
    int current_node_id;             // Current station/junction
    int next_node_id;                // Next destination
    std::vector<int> route_path;     // Full planned route
    double distance_to_next_m;       // Distance to next station
    
    // Safety & monitoring
    double time_since_last_brake_s;  // Elapsed time since last braking event
    bool is_faulty;                  // System fault flag
    std::string fault_description;   // Fault details
    
    /**
     * @brief Calculate braking distance using kinematic equation
     * @param current_velocity Current speed (m/s)
     * @param target_velocity Target speed (m/s)
     * @param deceleration Deceleration rate (m/s²)
     * @return Required distance in meters
     */
    double calculateBrakingDistance(double current_velocity, double target_velocity, double deceleration) const;
    
    /**
     * @brief Update train state based on current conditions
     */
    void updateState();
    
public:
    /**
     * @brief Constructor
     * @param id Unique train identifier
     * @param name Train name/label
     * @param train_type Type of train (Passenger, Freight, etc.)
     */
    Train(int id, const std::string& name, TrainType train_type);
    
    /**
     * @brief Destructor
     */
    ~Train() = default;
    
    // ============ Getters ============
    
    int getTrainID() const { return train_id; }
    const std::string& getTrainName() const { return train_name; }
    TrainType getTrainType() const { return type; }
    TrainState getTrainState() const { return state; }
    double getMass() const { return mass_kg; }
    double getLength() const { return length_meters; }
    double getMaxSpeed() const { return max_speed_ms; }
    double getCurrentVelocity() const { return physics.velocity_ms; }
    double getCurrentAcceleration() const { return physics.acceleration_ms2; }
    double getCurrentPosition() const { return physics.position_meters; }
    double getTargetVelocity() const { return physics.target_velocity_ms; }
    int getCurrentNode() const { return current_node_id; }
    int getNextNode() const { return next_node_id; }
    double getDistanceToNext() const { return distance_to_next_m; }
    bool isFaulty() const { return is_faulty; }
    const std::string& getFaultDescription() const { return fault_description; }
    const std::vector<int>& getRoute() const { return route_path; }
    const PhysicsState& getPhysicsState() const { return physics; }
    
    // ============ Setters ============
    
    /**
     * @brief Set the train's current position on track
     * @param position Position in meters
     */
    void setPosition(double position);
    
    /**
     * @brief Set the target velocity (desired speed)
     * @param velocity Target velocity in m/s
     */
    void setTargetVelocity(double velocity);
    
    /**
     * @brief Set the current route
     * @param path Vector of node IDs representing the route
     */
    void setRoute(const std::vector<int>& path);
    
    /**
     * @brief Set current and next nodes
     * @param current Current node ID
     * @param next Next node ID
     */
    void setCurrentNode(int current, int next);
    
    /**
     * @brief Initialize train properties based on type
     */
    void initializeProperties();
    
    // ============ Physics Simulation ============
    
    /**
     * @brief Update train physics for one timestep (50ms)
     * @param delta_time Time step in seconds
     */
    void updatePhysics(double delta_time);
    
    /**
     * @brief Apply braking force
     * @param braking_level Level of braking (0.0 to 1.0)
     */
    void applyBrake(double braking_level);
    
    /**
     * @brief Apply full emergency braking
     */
    void emergencyBrake();
    
    /**
     * @brief Apply acceleration
     * @param acceleration_level Level of acceleration (0.0 to 1.0)
     */
    void applyAcceleration(double acceleration_level);
    
    /**
     * @brief Hold train at current position (zero velocity)
     */
    void hold();
    
    // ============ Safety & Status ============
    
    /**
     * @brief Check if train is moving
     * @return true if velocity > 0.1 m/s
     */
    bool isMoving() const { return physics.velocity_ms > 0.1; }
    
    /**
     * @brief Set fault status
     * @param faulty Fault state
     * @param description Fault description
     */
    void setFault(bool faulty, const std::string& description = "");
    
    /**
     * @brief Clear fault status
     */
    void clearFault();
    
    /**
     * @brief Get string representation of train state
     * @return State as human-readable string
     */
    std::string getStateString() const;
    
    /**
     * @brief Get string representation of train type
     * @return Type as human-readable string
     */
    std::string getTypeString() const;
    
    /**
     * @brief Reset train to initial state
     */
    void reset();
};

} // namespace train

#endif // TRAIN_TRAIN_H

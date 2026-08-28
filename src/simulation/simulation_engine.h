#ifndef SIMULATION_SIMULATION_ENGINE_H
#define SIMULATION_SIMULATION_ENGINE_H

#include <vector>
#include <memory>
#include "../railway/graph.h"
#include "../train/train.h"

namespace simulation {

/**
 * @class SimulationEngine
 * @brief Core simulation loop and train movement orchestration
 * 
 * WEEK 1 DELIVERABLE: Class declaration only
 * TODO: Implement in Week 2-3
 */
class SimulationEngine {
private:
    railway::Graph network;
    std::vector<std::shared_ptr<train::Train>> trains;
    double current_time_s;
    bool is_running;
    
public:
    /**
     * @brief Initialize simulation
     */
    void initialize();
    
    /**
     * @brief Add train to simulation
     */
    void addTrain(std::shared_ptr<train::Train> train);
    
    /**
     * @brief Execute one simulation timestep (50ms)
     */
    void step();
    
    /**
     * @brief Run simulation for specified duration
     */
    void run(double duration_s);
    
    /**
     * @brief Stop simulation
     */
    void stop();
    
    /**
     * @brief Get current simulation time
     */
    double getCurrentTime() const { return current_time_s; }
};

} // namespace simulation

#endif // SIMULATION_SIMULATION_ENGINE_H

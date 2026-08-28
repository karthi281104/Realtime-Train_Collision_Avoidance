#ifndef MONITORING_DASHBOARD_H
#define MONITORING_DASHBOARD_H

#include <vector>
#include <string>
#include "../train/train.h"

namespace monitoring {

/**
 * @class Dashboard
 * @brief Real-time monitoring and visualization interface
 * 
 * WEEK 1 DELIVERABLE: Class declaration only
 * TODO: Implement in Week 2-3
 */
class Dashboard {
private:
    std::vector<std::string> event_log;
    bool is_active;
    
public:
    /**
     * @brief Initialize dashboard
     */
    void initialize();
    
    /**
     * @brief Update dashboard with current train states
     */
    void update(const std::vector<std::shared_ptr<train::Train>>& trains);
    
    /**
     * @brief Log an event
     */
    void logEvent(const std::string& event);
    
    /**
     * @brief Display dashboard
     */
    void display();
    
    /**
     * @brief Close dashboard
     */
    void close();
};

} // namespace monitoring

#endif // MONITORING_DASHBOARD_H

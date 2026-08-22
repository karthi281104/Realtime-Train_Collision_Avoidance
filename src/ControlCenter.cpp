#include "ControlCenter.hpp"
#include "core/Logger.hpp"
#include <iostream>
#include <sstream>

namespace tca {

ControlCenter::ControlCenter()
    : rm_(net_),
      cm_(net_),
      sim_(tsm_, net_, rm_, cm_, cr_, el_),
      dash_(tsm_, cm_, net_, sim_.clock()),
      scenarios_(tsm_, net_, rm_)
{}

ControlCenter::~ControlCenter() {
    stop();
    Logger::instance().shutdown();
}

bool ControlCenter::init(const std::string& configPath, const std::string& scenario,
                         std::size_t trainCount) {
    Logger::instance().init("logs", LogLevel::INFO);
    LOG_INFO("=== Train Collision Avoidance System Starting ===");

    if(!configPath.empty()) {
        if(!cfg_.load(configPath))
            LOG_WARN("Config file not found: " << configPath << " – using defaults");
        else
            LOG_INFO("Config loaded from " << configPath);
    }

    el_.init("logs");

    LOG_INFO("Loading scenario: " << scenario);
    auto sc = scenarios_.load(scenario, trainCount);
    LOG_INFO("Scenario: " << sc.name << " | " << sc.description);

    net_.print();
    tsm_.print();
    rm_.print();

    LOG_INFO("System initialized successfully.");
    return true;
}

void ControlCenter::run() {
    LOG_INFO("Starting real-time simulation...");
    dash_.start();
    sim_.start();

    LOG_INFO("System running. Press Enter to stop.");
    std::cin.get();

    stop();
}

void ControlCenter::stop() {
    sim_.stop();
    dash_.stop();
    el_.flush();
    LOG_INFO("System shutdown complete.");
}

void ControlCenter::runScenario(const std::string& name, double seconds, std::size_t trainCount) {
    Logger::instance().init("logs", LogLevel::INFO);
    el_.init("logs");

    auto sc = scenarios_.load(name, trainCount);
    LOG_INFO("Running scenario '" << sc.name << "' for " << seconds << "s");
    net_.print();
    tsm_.print();
    rm_.print();

    sim_.runBlocking(seconds);

    std::cout << "\n=== Scenario '" << sc.name << "' complete ===\n";
    tsm_.print();
    std::cout << "Logs written to ./logs/\n";
}

} // namespace tca

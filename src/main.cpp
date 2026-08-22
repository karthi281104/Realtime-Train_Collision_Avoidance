#include "ControlCenter.hpp"
#include "core/Logger.hpp"
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

static volatile std::sig_atomic_t gStop = 0;

void sigHandler(int) {
    gStop = 1;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    std::string scenario  = "rear_end";
    std::string config    = "config/system.cfg";
    double      duration  = 30.0;
    std::size_t trainCount = 0;
    bool        realtime  = false;

    for(int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if(arg == "--scenario" && i+1 < argc) scenario = argv[++i];
        if(arg == "--config"   && i+1 < argc) config   = argv[++i];
        if(arg == "--duration" && i+1 < argc) duration = std::stod(argv[++i]);
        if(arg == "--realtime")               realtime  = true;
        if(arg == "--help") {
            std::cout <<
                "Usage: train_sim [OPTIONS]\n"
                "  --scenario  <name>   Scenario name (default: rear_end)\n"
                "                       Available: normal, rear_end, head_on, junction,\n"
                "                                  comm_delay, packet_loss, sensor_fault,\n"
                "                                  multi_conflict, emergency, high_density\n"
                "  --config    <path>   Config file path (default: config/system.cfg)\n"
                "  --duration  <secs>   Simulation duration in seconds (default: 30)\n"
                "  --trains    <count>  Train count for high_density (default: 10)\n"
                "  --realtime           Run with real-time multithreaded dashboard\n"
                "  --help               Show this help\n";
            return 0;
        }
        if(arg == "--trains"   && i+1 < argc) trainCount = std::stoull(argv[++i]);
    }

    tca::ControlCenter cc;
    if(!cc.init(config, scenario, trainCount)) {
        std::cerr << "Initialization failed.\n";
        return 1;
    }

    if(realtime) {
        cc.run();
    } else {
        cc.runScenario(scenario, duration, trainCount);
    }

    return 0;
}

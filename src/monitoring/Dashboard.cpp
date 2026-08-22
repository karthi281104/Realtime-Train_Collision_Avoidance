#include "monitoring/Dashboard.hpp"
#include <chrono>
#include <cstdio>
#include <format>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace tca {

// ANSI colour codes
namespace ansi {
    constexpr auto RESET  = "\033[0m";
    constexpr auto BOLD   = "\033[1m";
    constexpr auto RED    = "\033[31m";
    constexpr auto GREEN  = "\033[32m";
    constexpr auto YELLOW = "\033[33m";
    constexpr auto CYAN   = "\033[36m";
    constexpr auto WHITE  = "\033[37m";
    constexpr auto BG_DARK= "\033[40m";
}

Dashboard::Dashboard(TrainStateManager& tsm,
                     ConflictManager&   cm,
                     RailwayNetwork&    net,
                     SimulationClock&   clk)
    : tsm_(tsm), cm_(cm), net_(net), clk_(clk)
{}

void Dashboard::start() {
    running_.store(true);
    thread_ = std::thread(&Dashboard::loop, this);
}

void Dashboard::stop() {
    running_.store(false);
    if(thread_.joinable()) thread_.join();
}

void Dashboard::loop() {
    using namespace std::chrono_literals;
    while(running_.load()) {
        render();
        std::this_thread::sleep_for(100ms);   // 10 Hz
    }
}

void Dashboard::clearScreen() {
    // ANSI escape: move cursor to top-left
    std::cout << "\033[H\033[J";
}

void Dashboard::render() {
    clearScreen();
    printHeader();
    printTrains();
    printConflicts();
    printFooter();
    std::cout << std::flush;
}

void Dashboard::printHeader() {
    using namespace ansi;
    double st = clk_.simTime();
    std::cout << BOLD << CYAN
              << "╔══════════════════════════════════════════════════════════════════╗\n"
              << "║       REAL-TIME TRAIN COLLISION AVOIDANCE SYSTEM  v0.1          ║\n"
              << "╠══════════════════════════════════════════════════════════════════╣\n"
              << "║ " << RESET << BOLD
              << std::format(" SimTime: {:>8.2f}s   Trains: {:>3}   Tick: {:>6}",
                             st,
                             tsm_.count(),
                             clk_.tickCount())
              << CYAN << "            ║\n"
              << "╠══════════════════════════════════════════════════════════════════╣\n"
              << RESET;
}

void Dashboard::printTrains() {
    using namespace ansi;
    std::cout << BOLD << "  ID   Name       Type        Pos(m)   Speed(km/h)  State\n" << RESET;
    std::cout << "  " << std::string(66, '-') << "\n";

    auto snap = tsm_.snapshot();
    for(auto& d : snap) {
        const char* col = RESET;
        if(d.state == TrainState::EMERGENCY_BRAKING) col = RED;
        else if(d.state == TrainState::BRAKING)      col = YELLOW;
        else if(d.state == TrainState::WARNING)      col = YELLOW;
        else if(d.state == TrainState::RUNNING)      col = GREEN;

        std::cout << col
                  << std::format("  {:>3}  {:>10}  {:>10}  {:>7.0f}  {:>10.1f}   {:<12}\n",
                                 d.id,
                                 d.name,
                                 toString(d.type),
                                 d.positionM,
                                 d.velocityMs * kMsToKmh,
                                 toString(d.state))
                  << RESET;
    }
}

void Dashboard::printConflicts() {
    using namespace ansi;
    auto conflicts = cm_.activeConflicts();
    std::cout << "\n" << BOLD << "  ACTIVE CONFLICTS (" << conflicts.size() << ")\n" << RESET;
    if(conflicts.empty()) {
        std::cout << GREEN << "  ✓ No conflicts detected.\n" << RESET;
        return;
    }
    for(auto& ci : conflicts) {
        const char* col = (ci.risk == RiskLevel::EMERGENCY) ? RED :
                          (ci.risk == RiskLevel::CRITICAL)  ? RED :
                          (ci.risk == RiskLevel::WARNING)   ? YELLOW : CYAN;
        std::cout << col
                  << std::format("  [{:<9}] T{:>2}<->T{:>2}  TTC={:>6.1f}s  Sep={:>6.0f}m  {}\n",
                                 toString(ci.risk),
                                 ci.trainA, ci.trainB,
                                 ci.ttcSeconds,
                                 ci.separationM,
                                 toString(ci.recommendedAction))
                  << RESET;
    }
}

void Dashboard::printFooter() {
    using namespace ansi;
    std::cout << CYAN
              << "╚══════════════════════════════════════════════════════════════════╝\n"
              << RESET << "  Press Ctrl+C to stop.\n";
}

} // namespace tca

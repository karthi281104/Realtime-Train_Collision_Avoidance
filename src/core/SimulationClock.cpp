#include "core/SimulationClock.hpp"
#include <thread>

namespace tca {

SimulationClock::SimulationClock(double dt)
    : dt_(dt), startWall_(WallClock::now())
{}

void SimulationClock::tick() {
    auto prevTick = tick_.load(std::memory_order_relaxed);
    double nextSimT = (prevTick + 1) * dt_;

    // Wall time we should reach before returning
    auto targetWall = startWall_ + std::chrono::duration_cast<WallClock::duration>(
                          std::chrono::duration<double>(nextSimT));

    auto now = WallClock::now();
    if(now < targetWall)
        std::this_thread::sleep_until(targetWall);

    tick_.fetch_add(1, std::memory_order_relaxed);
    simTime_.store(nextSimT, std::memory_order_relaxed);
}

void SimulationClock::reset() {
    tick_.store(0, std::memory_order_relaxed);
    simTime_.store(0.0, std::memory_order_relaxed);
    startWall_ = WallClock::now();
}

double SimulationClock::wallElapsed() const {
    Duration d = WallClock::now() - startWall_;
    return d.count();
}

} // namespace tca

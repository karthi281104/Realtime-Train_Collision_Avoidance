#pragma once
#include "core/Types.hpp"
#include <atomic>
#include <mutex>

namespace tca {

/// Central fixed-timestep simulation clock.
/// Simulation time advances in exact Δt increments.
class SimulationClock {
public:
    explicit SimulationClock(double dt = kDefaultDt);

    /// Advance by one tick; blocks if running ahead of wall time.
    void tick();

    /// Reset to t=0.
    void reset();

    double   simTime()  const noexcept { return simTime_.load(std::memory_order_relaxed); }
    double   dt()       const noexcept { return dt_; }
    uint64_t tickCount()const noexcept { return tick_.load(std::memory_order_relaxed); }

    /// Wall-clock elapsed since clock started.
    double wallElapsed() const;

private:
    double               dt_;
    std::atomic<double>  simTime_{0.0};
    std::atomic<uint64_t>tick_{0};
    TimePoint            startWall_;
    mutable std::mutex   mtx_;
};

} // namespace tca

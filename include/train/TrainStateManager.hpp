#pragma once
#include "train/TrainTypes.hpp"
#include "train/TrainSubtypes.hpp"
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace tca {

/// Owns all Train objects; provides thread-safe snapshot interface.
class TrainStateManager {
public:
    TrainStateManager() = default;

    // ── Lifetime ─────────────────────────────────────────────────────────────
    void addTrain(std::unique_ptr<Train> t);
    void removeTrain(TrainId id);
    void clear() {
        std::lock_guard lk(mtx_);
        trains_.clear();
    }

    // ── Raw access (call under lock) ──────────────────────────────────────────
    Train*       get(TrainId id);
    const Train* get(TrainId id) const;

    /// Execute fn(Train&) for every train under a single lock.
    void forEach(std::invocable<Train&> auto fn) {
        std::lock_guard lk(mtx_);
        for(auto& [id, t] : trains_) fn(*t);
    }

    void forEachConst(std::invocable<const Train&> auto fn) const {
        std::lock_guard lk(mtx_);
        for(auto& [id, t] : trains_) fn(*t);
    }

    // ── Snapshot (immutable copy for prediction thread) ───────────────────────
    std::vector<TrainData> snapshot() const;

    std::size_t count() const;

    // ── Helpers ───────────────────────────────────────────────────────────────
    void print() const;

private:
    mutable std::mutex                           mtx_;
    std::unordered_map<TrainId, std::unique_ptr<Train>> trains_;
};

} // namespace tca

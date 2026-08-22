#include "train/TrainStateManager.hpp"
#include "core/Logger.hpp"
#include <iostream>
#include <sstream>

namespace tca {

void TrainStateManager::addTrain(std::unique_ptr<Train> t) {
    std::lock_guard lk(mtx_);
    TrainId id = t->id();
    trains_[id] = std::move(t);
}

void TrainStateManager::removeTrain(TrainId id) {
    std::lock_guard lk(mtx_);
    trains_.erase(id);
}

Train* TrainStateManager::get(TrainId id) {
    std::lock_guard lk(mtx_);
    auto it = trains_.find(id);
    return it != trains_.end() ? it->second.get() : nullptr;
}

const Train* TrainStateManager::get(TrainId id) const {
    std::lock_guard lk(mtx_);
    auto it = trains_.find(id);
    return it != trains_.end() ? it->second.get() : nullptr;
}

std::vector<TrainData> TrainStateManager::snapshot() const {
    std::lock_guard lk(mtx_);
    std::vector<TrainData> snap;
    snap.reserve(trains_.size());
    for(auto& [id, t] : trains_)
        snap.push_back(t->data());
    return snap;
}

std::size_t TrainStateManager::count() const {
    std::lock_guard lk(mtx_);
    return trains_.size();
}

void TrainStateManager::print() const {
    std::lock_guard lk(mtx_);
    std::cout << "\n═══ TRAIN FLEET (" << trains_.size() << " trains) ═══\n";
    for(auto& [id, t] : trains_) {
        const auto& d = t->data();
        std::cout << "  " << d.name
                  << " [" << toString(d.type) << "]"
                  << "  pos=" << static_cast<int>(d.positionM) << "m"
                  << "  v=" << static_cast<int>(d.velocityMs * kMsToKmh) << "km/h"
                  << "  state=" << toString(d.state) << "\n";
    }
}

} // namespace tca

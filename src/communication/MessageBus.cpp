#include "communication/MessageBus.hpp"
#include "core/Logger.hpp"

namespace tca {

void MessageBus::registerTrain(TrainId id, double delayMs, double jitterMs, double lossRate) {
    channels_.emplace(std::piecewise_construct,
                      std::forward_as_tuple(id),
                      std::forward_as_tuple(delayMs, jitterMs, lossRate, id));
}

void MessageBus::broadcast(const TrainData& state, double simNow, double wallNow) {
    auto it = channels_.find(state.id);
    if(it != channels_.end()) it->second.send(state, simNow, wallNow);
}

void MessageBus::processIncoming(TrainStateManager& tsm, double wallNow) {
    for(auto& [id, ch] : channels_) {
        auto msgs = ch.receive(wallNow);
        for(auto& msg : msgs) {
            Train* t = tsm.get(msg.id);
            if(!t) continue;
            auto& d = t->data();
            // Only update if message is newer than current
            if(msg.simTimestamp > d.simTimestamp) {
                d.positionM   = msg.positionM;
                d.velocityMs  = msg.velocityMs;
                d.simTimestamp = msg.simTimestamp;
                d.lastCommTime = wallNow;
            }
        }
    }
}

CommChannel* MessageBus::channel(TrainId id) {
    auto it = channels_.find(id);
    return it != channels_.end() ? &it->second : nullptr;
}

void MessageBus::setFault(TrainId id, SensorStatus s) {
    sensorOverride_[id] = s;
    if(s == SensorStatus::FAILED) {
        if(auto* ch = channel(id)) ch->setLossRate(1.0);  // total loss
    } else if(s == SensorStatus::DEGRADED) {
        if(auto* ch = channel(id)) { ch->setDelayMs(500); ch->setLossRate(0.3); }
    }
}

} // namespace tca

#pragma once
#include "communication/CommChannel.hpp"
#include "train/TrainStateManager.hpp"
#include <unordered_map>

namespace tca {

/// Central bus: each train has its own CommChannel.
class MessageBus {
public:
    void registerTrain(TrainId id, double delayMs = 100.0,
                       double jitterMs = 30.0, double lossRate = 0.02);

    void broadcast(const TrainData& state, double simNow);

    /// Apply arrived messages to TrainStateManager.
    void processIncoming(TrainStateManager& tsm, double wallNow);

    CommChannel* channel(TrainId id);
    void setFault(TrainId id, SensorStatus s);

private:
    std::unordered_map<TrainId, CommChannel> channels_;
    std::unordered_map<TrainId, SensorStatus> sensorOverride_;
};

} // namespace tca

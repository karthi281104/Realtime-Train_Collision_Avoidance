#include "train/TrainSubtypes.hpp"

namespace tca {

// ─── PassengerTrain ───────────────────────────────────────────────────────────
PassengerTrain::PassengerTrain(TrainId id, const std::string& name,
                               uint32_t from, uint32_t to,
                               double startPosM, double initSpeedKmh) 
    : Train([&]{
        TrainData d;
        d.id = id; d.type = TrainType::PASSENGER; d.name = name;
        d.spec = TrainSpec::forPassenger();
        d.positionM = startPosM;
        d.velocityMs = initSpeedKmh * kMsToMs;
        d.targetSpeedMs = d.velocityMs;
        d.currentFromNode = from; d.currentToNode = to;
        d.destinationNode = to;
        d.direction = Direction::FORWARD;
        d.state = (d.velocityMs > 0) ? TrainState::RUNNING : TrainState::IDLE;
        return d;
    }())
{}

// ─── ExpressTrain ─────────────────────────────────────────────────────────────
ExpressTrain::ExpressTrain(TrainId id, const std::string& name,
                           uint32_t from, uint32_t to,
                           double startPosM, double initSpeedKmh)
    : Train([&]{
        TrainData d;
        d.id = id; d.type = TrainType::EXPRESS; d.name = name;
        d.spec = TrainSpec::forExpress();
        d.positionM = startPosM;
        d.velocityMs = initSpeedKmh * kMsToMs;
        d.targetSpeedMs = d.velocityMs;
        d.currentFromNode = from; d.currentToNode = to;
        d.destinationNode = to;
        d.direction = Direction::FORWARD;
        d.state = (d.velocityMs > 0) ? TrainState::RUNNING : TrainState::IDLE;
        return d;
    }())
{}

void ExpressTrain::applyPhysics(double dt) {
    // Express trains use a tighter control loop — no special override needed
    Train::applyPhysics(dt);
}

// ─── FreightTrain ─────────────────────────────────────────────────────────────
FreightTrain::FreightTrain(TrainId id, const std::string& name,
                           uint32_t from, uint32_t to,
                           double startPosM, double initSpeedKmh)
    : Train([&]{
        TrainData d;
        d.id = id; d.type = TrainType::FREIGHT; d.name = name;
        d.spec = TrainSpec::forFreight();
        d.positionM = startPosM;
        d.velocityMs = initSpeedKmh * kMsToMs;
        d.targetSpeedMs = d.velocityMs;
        d.currentFromNode = from; d.currentToNode = to;
        d.destinationNode = to;
        d.direction = Direction::FORWARD;
        d.state = (d.velocityMs > 0) ? TrainState::RUNNING : TrainState::IDLE;
        return d;
    }())
{}

} // namespace tca

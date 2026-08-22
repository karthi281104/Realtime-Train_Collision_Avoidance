#pragma once
#include "train/TrainTypes.hpp"

namespace tca {

class PassengerTrain : public Train {
public:
    explicit PassengerTrain(TrainId id, const std::string& name,
                            uint32_t fromNode, uint32_t toNode,
                            double startPosM = 0.0, double initSpeedKmh = 0.0);
};

class ExpressTrain : public Train {
public:
    explicit ExpressTrain(TrainId id, const std::string& name,
                          uint32_t fromNode, uint32_t toNode,
                          double startPosM = 0.0, double initSpeedKmh = 0.0);
    void applyPhysics(double dt) override;
};

class FreightTrain : public Train {
public:
    explicit FreightTrain(TrainId id, const std::string& name,
                          uint32_t fromNode, uint32_t toNode,
                          double startPosM = 0.0, double initSpeedKmh = 0.0);
};

} // namespace tca

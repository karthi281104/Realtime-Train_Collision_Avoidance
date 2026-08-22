#pragma once
#include "train/TrainTypes.hpp"
#include "railway/RailwayNetwork.hpp"
#include <vector>

namespace tca {

enum class ConflictType : uint8_t {
    REAR_END,
    HEAD_ON,
    JUNCTION,
    UNDEFINED
};
std::string_view toString(ConflictType ct);

struct ConflictInfo {
    ConflictId   id{0};
    TrainId      trainA{0};
    TrainId      trainB{0};
    ConflictType type{ConflictType::UNDEFINED};
    RiskLevel    risk{RiskLevel::SAFE};
    double       ttcSeconds{kInfinity};
    double       separationM{kInfinity};
    double       requiredSepM{0.0};
    double       detectedAtTime{0.0};   // sim time

    ConflictAction recommendedAction{ConflictAction::NONE};
};

struct PredictionResult {
    bool          hasConflict{false};
    ConflictInfo  conflict;
    double        minSeparationM{kInfinity};
    double        timeOfMinSep{0.0};
};

class CollisionPredictor {
public:
    explicit CollisionPredictor(const RailwayNetwork& net,
                                double predHorizonS = kPredHorizon,
                                double dtPredS      = kDefaultDt);

    /// Check a pair of trains for potential conflict.
    PredictionResult predict(const TrainData& a,
                             const TrainData& b,
                             double           simNow) const;

    /// Generate candidate conflict pairs from a snapshot
    /// (only pairs on same/connected tracks).
    std::vector<std::pair<std::size_t,std::size_t>>
    candidatePairs(const std::vector<TrainData>& snapshot) const;

    void setHorizon(double s) { horizon_ = s; }

private:
    PredictionResult checkSameTrack   (const TrainData& a, const TrainData& b, double t) const;
    PredictionResult checkHeadOn      (const TrainData& a, const TrainData& b, double t) const;
    PredictionResult checkJunction    (const TrainData& a, const TrainData& b, double t) const;

    RiskLevel assessRisk(double ttc, double minSep, double reqSep,
                         double relVel) const;

    const RailwayNetwork& net_;
    double horizon_;
    double dtPred_;
};

} // namespace tca

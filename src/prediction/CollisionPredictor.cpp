#include "prediction/CollisionPredictor.hpp"
#include "train/TrainPhysics.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace tca {

std::string_view toString(ConflictType ct) {
    switch(ct) {
        case ConflictType::REAR_END:  return "REAR_END";
        case ConflictType::HEAD_ON:   return "HEAD_ON";
        case ConflictType::JUNCTION:  return "JUNCTION";
        default:                       return "UNDEFINED";
    }
}

CollisionPredictor::CollisionPredictor(const RailwayNetwork& net,
                                       double predHorizonS,
                                       double dtPredS)
    : net_(net), horizon_(predHorizonS), dtPred_(dtPredS)
{}

// ─── Main entry ───────────────────────────────────────────────────────────────

PredictionResult CollisionPredictor::predict(const TrainData& a,
                                              const TrainData& b,
                                              double simNow) const {
    // Same track?
    if(a.currentTrackId == b.currentTrackId && a.currentTrackId != 0) {
        bool sameDir = (a.direction == b.direction);
        if(sameDir) return checkSameTrack(a, b, simNow);
        else        return checkHeadOn   (a, b, simNow);
    }
    // Adjacent/junction check
    return checkJunction(a, b, simNow);
}

// ─── Same track, same direction ───────────────────────────────────────────────

PredictionResult CollisionPredictor::checkSameTrack(const TrainData& a,
                                                     const TrainData& b,
                                                     double simNow) const {
    // Determine leading/trailing
    const TrainData* leading  = (a.positionM > b.positionM) ? &a : &b;
    const TrainData* trailing = (a.positionM > b.positionM) ? &b : &a;

    double reqSep = physics::requiredSeparation(*trailing);

    PredictionResult res;
    res.minSeparationM = kInfinity;

    // Sample over prediction horizon
    double posL = leading->positionM,  velL = leading->velocityMs;
    double posT = trailing->positionM, velT = trailing->velocityMs;
    double accelL = physics::computeAcceleration(*leading);
    double accelT = physics::computeAcceleration(*trailing);

    for(double t = 0.0; t <= horizon_; t += dtPred_) {
        double sep = posL - posT;
        if(sep < res.minSeparationM) {
            res.minSeparationM = sep;
            res.timeOfMinSep   = simNow + t;
        }
        // Step
        physics::integrate(posL, velL, accelL, dtPred_);
        physics::integrate(posT, velT, accelT, dtPred_);
        // Clamp speed
        velL = std::max(0.0, std::min(velL, leading->spec.maxSpeedMs));
        velT = std::max(0.0, std::min(velT, trailing->spec.maxSpeedMs));
    }

    double relVel = trailing->velocityMs - leading->velocityMs;
    double ttc    = physics::ttcSameDirection(
                        leading->positionM, leading->velocityMs,
                        trailing->positionM, trailing->velocityMs,
                        reqSep);

    if(res.minSeparationM < reqSep) {
        ConflictInfo ci;
        ci.trainA        = a.id;
        ci.trainB        = b.id;
        ci.type          = ConflictType::REAR_END;
        ci.ttcSeconds    = ttc;
        ci.separationM   = leading->positionM - trailing->positionM;
        ci.requiredSepM  = reqSep;
        ci.detectedAtTime= simNow;
        ci.risk          = assessRisk(ttc, res.minSeparationM, reqSep, relVel);
        ci.recommendedAction = (ci.risk >= RiskLevel::CRITICAL)
                               ? ConflictAction::CONTROLLED_BRAKING
                               : ConflictAction::SPEED_RESTRICTION;

        res.hasConflict = true;
        res.conflict    = ci;
    }
    return res;
}

// ─── Head-on ─────────────────────────────────────────────────────────────────

PredictionResult CollisionPredictor::checkHeadOn(const TrainData& a,
                                                  const TrainData& b,
                                                  double simNow) const {
    double sepReqA = physics::requiredSeparation(a);
    double sepReqB = physics::requiredSeparation(b);
    double reqSep  = sepReqA + sepReqB;

    double currentSep = std::abs(a.positionM - b.positionM);
    double ttc = physics::ttcHeadOn(
        std::min(a.positionM, b.positionM), a.velocityMs,
        std::max(a.positionM, b.positionM), b.velocityMs);

    double relVel = a.velocityMs + b.velocityMs;

    PredictionResult res;
    res.minSeparationM = std::max(0.0, currentSep - relVel * std::min(ttc, horizon_));

    if(res.minSeparationM < reqSep) {
        ConflictInfo ci;
        ci.trainA       = a.id; ci.trainB = b.id;
        ci.type         = ConflictType::HEAD_ON;
        ci.ttcSeconds   = ttc;
        ci.separationM  = currentSep;
        ci.requiredSepM = reqSep;
        ci.detectedAtTime = simNow;
        ci.risk         = assessRisk(ttc, res.minSeparationM, reqSep, relVel);
        ci.recommendedAction = ConflictAction::EMERGENCY_BRAKING;
        res.hasConflict = true;
        res.conflict    = ci;
    }
    return res;
}

// ─── Junction conflict ────────────────────────────────────────────────────────

PredictionResult CollisionPredictor::checkJunction(const TrainData& a,
                                                    const TrainData& b,
                                                    double simNow) const {
    // Check if both trains' destination nodes are the same junction
    if(a.currentToNode != b.currentToNode) return {};

    auto* nd = net_.node(a.currentToNode);
    if(!nd || !nd->isJunction) return {};

    // Time for each to reach junction
    auto timeToJunc = [this](const TrainData& td) -> double {
        if(td.velocityMs <= kEpsilon) return kInfinity;
        double remDist = 500.0;
        if(auto* tk = net_.track(td.currentTrackId)) {
            remDist = std::max(0.0, tk->lengthM - td.positionM);
        }
        return remDist / td.velocityMs;
    };

    double tA = timeToJunc(a), tB = timeToJunc(b);
    double overlap = std::abs(tA - tB);

    PredictionResult res;
    if(overlap < 5.0 && tA < 30.0 && tB < 30.0) {
        ConflictInfo ci;
        ci.trainA = a.id; ci.trainB = b.id;
        ci.type   = ConflictType::JUNCTION;
        ci.ttcSeconds   = std::min(tA, tB);
        ci.separationM  = 0.0;
        ci.requiredSepM = 200.0;
        ci.detectedAtTime = simNow;
        ci.risk = RiskLevel::WARNING;
        ci.recommendedAction = ConflictAction::TRAIN_HOLD;
        res.hasConflict = true;
        res.conflict    = ci;
    }
    return res;
}

// ─── Risk assessment ─────────────────────────────────────────────────────────

RiskLevel CollisionPredictor::assessRisk(double ttc, double minSep,
                                          double reqSep, double relVel) const {
    if(ttc < 5.0  || minSep < 0.0)           return RiskLevel::EMERGENCY;
    if(ttc < 10.0 || minSep < reqSep * 0.25) return RiskLevel::CRITICAL;
    if(ttc < 20.0 || minSep < reqSep * 0.5)  return RiskLevel::WARNING;
    if(ttc < 30.0 || minSep < reqSep * 0.75) return RiskLevel::CAUTION;
    return RiskLevel::SAFE;
}

// ─── Candidate pairs ─────────────────────────────────────────────────────────

std::vector<std::pair<std::size_t,std::size_t>>
CollisionPredictor::candidatePairs(const std::vector<TrainData>& snap) const {
    std::vector<std::pair<std::size_t,std::size_t>> pairs;
    for(std::size_t i = 0; i < snap.size(); ++i)
        for(std::size_t j = i+1; j < snap.size(); ++j) {
            const auto& a = snap[i]; const auto& b = snap[j];
            // Same track → always candidate
            if(a.currentTrackId == b.currentTrackId && a.currentTrackId != 0) {
                pairs.emplace_back(i,j); continue;
            }
            // Same junction destination → candidate
            if(a.currentToNode == b.currentToNode && a.currentToNode != 0) {
                auto* nd = net_.node(a.currentToNode);
                if(nd && nd->isJunction) pairs.emplace_back(i,j);
            }
        }
    return pairs;
}

} // namespace tca

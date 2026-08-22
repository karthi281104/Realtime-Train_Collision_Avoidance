#include "safety/ConflictManager.hpp"
#include "core/Logger.hpp"
#include <algorithm>
#include <sstream>

namespace tca {

// ─── RiskAssessor ─────────────────────────────────────────────────────────────

RiskLevel RiskAssessor::assess(const PredictionResult& pr) const {
    if(!pr.hasConflict) return RiskLevel::SAFE;
    return pr.conflict.risk;
}

double RiskAssessor::ttcThreshold(RiskLevel rl) const {
    switch(rl) {
        case RiskLevel::EMERGENCY: return 5.0;
        case RiskLevel::CRITICAL:  return 10.0;
        case RiskLevel::WARNING:   return 20.0;
        case RiskLevel::CAUTION:   return 30.0;
        default:                   return kInfinity;
    }
}

double RiskAssessor::sepThreshold(RiskLevel rl) const {
    switch(rl) {
        case RiskLevel::EMERGENCY: return 0.0;
        case RiskLevel::CRITICAL:  return 50.0;
        case RiskLevel::WARNING:   return 150.0;
        case RiskLevel::CAUTION:   return 300.0;
        default:                   return 0.0;
    }
}

// ─── ConflictManager ─────────────────────────────────────────────────────────

ConflictManager::ConflictManager(const RailwayNetwork& net)
    : predictor_(net)
{}

std::vector<ConflictInfo>
ConflictManager::update(const std::vector<TrainData>& snapshot, double simNow) {
    std::lock_guard lk(mtx_);
    active_.clear();

    auto pairs = predictor_.candidatePairs(snapshot);
    for(auto [i, j] : pairs) {
        auto pr = predictor_.predict(snapshot[i], snapshot[j], simNow);
        if(pr.hasConflict) {
            pr.conflict.id = nextId_++;
            active_.push_back(pr.conflict);

            std::ostringstream ss;
            ss << "CONFLICT#" << pr.conflict.id
               << " [" << toString(pr.conflict.type) << "]"
               << " T" << pr.conflict.trainA << " <-> T" << pr.conflict.trainB
               << " TTC=" << static_cast<int>(pr.conflict.ttcSeconds) << "s"
               << " Sep=" << static_cast<int>(pr.conflict.separationM) << "m"
               << " Risk=" << toString(pr.conflict.risk);
            LOG_WARN(ss.str());
        }
    }
    return active_;
}

const std::vector<ConflictInfo>& ConflictManager::activeConflicts() const {
    return active_;
}

void ConflictManager::clearResolved() {
    std::lock_guard lk(mtx_);
    active_.clear();
}

} // namespace tca

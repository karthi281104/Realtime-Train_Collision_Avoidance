#include "safety/ConflictResolver.hpp"
#include "train/TrainPhysics.hpp"
#include "prediction/CollisionPredictor.hpp"
#include "core/Logger.hpp"
#include <algorithm>
#include <sstream>

namespace tca {

std::vector<Resolution>
ConflictResolver::resolve(const std::vector<ConflictInfo>& conflicts,
                          TrainStateManager& tsm,
                          const RailwayNetwork& net,
                          double simNow) {
    std::vector<Resolution> resolutions;
    for(auto& ci : conflicts) {
        if(ci.type == ConflictType::HEAD_ON) {
            // Both trains in head-on must apply emergency braking
            for(TrainId tid : {ci.trainA, ci.trainB}) {
                Train* t = tsm.get(tid);
                if(t) {
                    Resolution res;
                    res.conflictId = ci.id;
                    res.targetTrain = tid;
                    res.action = ConflictAction::EMERGENCY_BRAKING;
                    res.newTargetSpeedMs = 0.0;
                    applyResolution(res, tsm);
                    resolutions.push_back(res);
                }
            }
        } else {
            auto res = resolveOne(ci, tsm, net, simNow);
            if(res.action != ConflictAction::NONE) {
                applyResolution(res, tsm);
                resolutions.push_back(res);
            }
        }
    }
    return resolutions;
}

Resolution ConflictResolver::resolveOne(const ConflictInfo& ci,
                                         TrainStateManager& tsm,
                                         const RailwayNetwork& net,
                                         double simNow) {
    // Always act on the trailing / higher-risk train
    TrainId targetId = ci.trainA;  // default: first train

    Train* tA = tsm.get(ci.trainA);
    Train* tB = tsm.get(ci.trainB);
    if(!tA || !tB) return {};

    // Choose which train is trailing (lower position = trailing)
    if(ci.type == ConflictType::REAR_END) {
        targetId = (tA->data().positionM < tB->data().positionM)
                   ? ci.trainA : ci.trainB;
    }

    Train* target = tsm.get(targetId);
    const Train* other  = tsm.get(targetId == ci.trainA ? ci.trainB : ci.trainA);
    if(!target || !other) return {};

    Resolution res;
    res.conflictId  = ci.id;
    res.targetTrain = targetId;

    // ── Safety hierarchy ──────────────────────────────────────────────────────
    // 1. Emergency
    if(ci.risk == RiskLevel::EMERGENCY) {
        res.action = ConflictAction::EMERGENCY_BRAKING;
        res.newTargetSpeedMs = 0.0;
        return res;
    }

    // 2. Critical → controlled braking
    if(ci.risk == RiskLevel::CRITICAL) {
        double safeSpeed = other->data().velocityMs * 0.8;
        if(simulateResolution(ci, target->data(), other->data(), net)) {
            res.action = ConflictAction::CONTROLLED_BRAKING;
            res.newTargetSpeedMs = safeSpeed;
        } else {
            res.action = ConflictAction::EMERGENCY_BRAKING;
            res.newTargetSpeedMs = 0.0;
        }
        return res;
    }

    // 3. Warning → speed restriction
    if(ci.risk == RiskLevel::WARNING) {
        double safeSpeed = std::max(0.0, other->data().velocityMs);
        res.action = ConflictAction::SPEED_RESTRICTION;
        res.newTargetSpeedMs = safeSpeed;
        return res;
    }

    // 4. Caution → advisory
    if(ci.risk == RiskLevel::CAUTION) {
        double safeSpeed = target->data().velocityMs * 0.9;
        res.action = ConflictAction::SPEED_ADVISORY;
        res.newTargetSpeedMs = safeSpeed;
        return res;
    }

    return res;
}

bool ConflictResolver::simulateResolution(const ConflictInfo& ci,
                                           const TrainData& modified,
                                           const TrainData& other,
                                           const RailwayNetwork& net) const {
    // Quick forward simulation: will the modified train be safe?
    double posM = modified.positionM, velM = modified.velocityMs;
    double posO = other.positionM,    velO = other.velocityMs;
    double accelO = physics::computeAcceleration(other);

    TrainData tmp = modified;
    tmp.targetSpeedMs = other.velocityMs * 0.8;
    double accelM = physics::computeAcceleration(tmp);

    double reqSep = physics::requiredSeparation(modified);

    for(double t = 0; t < 30.0; t += kDefaultDt) {
        physics::integrate(posM, velM, accelM, kDefaultDt);
        physics::integrate(posO, velO, accelO, kDefaultDt);
        velM = std::max(0.0, velM);
        velO = std::max(0.0, velO);
        double sep = std::abs(posO - posM);
        if(sep < reqSep * 0.5) return false;
    }
    return true;
}

void ConflictResolver::applyResolution(const Resolution& res, TrainStateManager& tsm) {
    Train* t = tsm.get(res.targetTrain);
    if(!t) return;

    auto& d = t->data();
    if(res.action == ConflictAction::EMERGENCY_BRAKING) {
        d.emergencyBrakeActive = true;
        d.targetSpeedMs = 0.0;
        d.state = TrainState::EMERGENCY_BRAKING;
    } else if(res.newTargetSpeedMs >= 0.0) {
        d.targetSpeedMs = res.newTargetSpeedMs;
        d.state = (d.state == TrainState::EMERGENCY_BRAKING)
                  ? TrainState::BRAKING : TrainState::BRAKING;
    }

    std::ostringstream ss;
    ss << "RESOLVED #" << res.conflictId
       << " → T" << res.targetTrain
       << " action=" << toString(res.action)
       << " targetSpeed=" << static_cast<int>(res.newTargetSpeedMs * kMsToKmh) << "km/h";
    LOG_INFO(ss.str());
}

} // namespace tca

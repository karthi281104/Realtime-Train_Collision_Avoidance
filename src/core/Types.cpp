#include "core/Types.hpp"
#include <string_view>

namespace tca {

std::string_view toString(TrainState s) {
    switch(s) {
        case TrainState::IDLE:             return "IDLE";
        case TrainState::RUNNING:          return "RUNNING";
        case TrainState::WARNING:          return "WARNING";
        case TrainState::BRAKING:          return "BRAKING";
        case TrainState::STOPPED:          return "STOPPED";
        case TrainState::EMERGENCY_BRAKING:return "EMRG_BRAKE";
    }
    return "UNKNOWN";
}

std::string_view toString(RiskLevel r) {
    switch(r) {
        case RiskLevel::SAFE:      return "SAFE";
        case RiskLevel::CAUTION:   return "CAUTION";
        case RiskLevel::WARNING:   return "WARNING";
        case RiskLevel::CRITICAL:  return "CRITICAL";
        case RiskLevel::EMERGENCY: return "EMERGENCY";
    }
    return "UNKNOWN";
}

std::string_view toString(ConflictAction a) {
    switch(a) {
        case ConflictAction::NONE:               return "NONE";
        case ConflictAction::SPEED_ADVISORY:     return "SPEED_ADVISORY";
        case ConflictAction::SPEED_RESTRICTION:  return "SPEED_RESTRICT";
        case ConflictAction::CONTROLLED_BRAKING: return "CTRL_BRAKE";
        case ConflictAction::TRAIN_HOLD:         return "TRAIN_HOLD";
        case ConflictAction::EMERGENCY_BRAKING:  return "EMRG_BRAKE";
        case ConflictAction::REROUTE:            return "REROUTE";
    }
    return "UNKNOWN";
}

} // namespace tca

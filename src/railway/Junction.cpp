#include "railway/Junction.hpp"
#include <string_view>

namespace tca {

std::string_view toString(JunctionType type) {
    switch(type) {
        case JunctionType::SIMPLE:  return "SIMPLE";
        case JunctionType::DIAMOND: return "DIAMOND";
        case JunctionType::SWITCH:  return "SWITCH";
        default:                    return "UNKNOWN";
    }
}

} // namespace tca

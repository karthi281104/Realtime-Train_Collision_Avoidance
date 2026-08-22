#include "railway/Signal.hpp"
#include <string_view>

namespace tca {

std::string_view toString(SignalAspect aspect) {
    switch(aspect) {
        case SignalAspect::GREEN:  return "GREEN";
        case SignalAspect::YELLOW: return "YELLOW";
        case SignalAspect::RED:    return "RED";
        default:                   return "UNKNOWN";
    }
}

} // namespace tca

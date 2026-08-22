#pragma once
#include "core/Types.hpp"
#include <string>

namespace tca {

struct Station {
    StationId   id{0};
    std::string name;
    double      posX{0.0};   // For visualisation / reference only
    double      posY{0.0};
    int         platform{1};
};

} // namespace tca

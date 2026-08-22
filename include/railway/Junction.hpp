#pragma once
#include "core/Types.hpp"
#include <string>
#include <vector>

namespace tca {

enum class JunctionType : uint8_t { SIMPLE, DIAMOND, SWITCH };

struct Junction {
    JunctionId  id{0};
    std::string name;
    JunctionType type{JunctionType::SIMPLE};
    std::vector<TrackId> connectedTracks;
    bool locked{false};   // true → no train may enter
    TrainId lockHolder{0};

    void lock  (TrainId tid) { locked = true;  lockHolder = tid; }
    void unlock()            { locked = false; lockHolder = 0;   }
};

std::string_view toString(JunctionType type);

} // namespace tca

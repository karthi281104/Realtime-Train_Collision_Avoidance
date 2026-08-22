#pragma once
#include "core/Types.hpp"
#include <string>
#include <vector>

namespace tca {

/// A directed track segment between two nodes.
struct Track {
    TrackId     id{0};
    std::string name;
    uint32_t    fromNode{0};   // node ID (station/junction)
    uint32_t    toNode{0};
    double      lengthM{0.0};  // metres
    double      speedLimitMs{0.0}; // m/s
    bool        bidirectional{true};
    bool        enabled{true};

    // Runtime occupancy: list of train IDs currently on this track
    std::vector<TrainId> occupants;

    bool isOccupied() const { return !occupants.empty(); }
    void addOccupant(TrainId tid);
    void removeOccupant(TrainId tid);
};

} // namespace tca

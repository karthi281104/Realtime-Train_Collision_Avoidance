#include "railway/Track.hpp"
#include <algorithm>

namespace tca {

void Track::addOccupant(TrainId tid) {
    if(std::find(occupants.begin(), occupants.end(), tid) == occupants.end())
        occupants.push_back(tid);
}

void Track::removeOccupant(TrainId tid) {
    std::erase(occupants, tid);
}

} // namespace tca

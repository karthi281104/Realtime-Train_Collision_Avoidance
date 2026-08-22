#pragma once
#include "core/Types.hpp"

namespace tca {

struct Signal {
    SignalId    id{0};
    TrackId     trackId{0};
    double      posOnTrackM{0.0};  // metres from track start
    SignalAspect aspect{SignalAspect::GREEN};
    bool        operational{true};

    void setAspect(SignalAspect a) { if(operational) aspect = a; }
};

} // namespace tca

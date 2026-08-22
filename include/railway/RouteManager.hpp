#pragma once
#include "railway/RailwayNetwork.hpp"
#include <vector>

namespace tca {

struct Route {
    RouteId               id{0};
    TrainId               trainId{0};
    uint32_t              srcNode{0};
    uint32_t              dstNode{0};
    std::vector<uint32_t> nodePath;
    std::vector<TrackId>  trackPath;
    int                   currentSegment{0}; // index in trackPath

    bool isComplete()  const { return currentSegment >= static_cast<int>(trackPath.size()); }
    TrackId currentTrack() const;
    TrackId nextTrack()    const;
    void    advanceSegment()   { ++currentSegment; }
};

class RouteManager {
public:
    explicit RouteManager(RailwayNetwork& net);

    /// Compute route using Dijkstra; returns RouteId (0 = failure).
    RouteId assignRoute(TrainId tid, uint32_t srcNode, uint32_t dstNode);

    Route*       route(RouteId id);
    const Route* route(RouteId id) const;
    Route*       routeForTrain(TrainId tid);

    void removeRoute(RouteId id);
    void print()     const;

private:
    RailwayNetwork&                      net_;
    RouteId                              nextId_{1};
    std::unordered_map<RouteId, Route>   routes_;
    std::unordered_map<TrainId, RouteId> trainRoute_;
};

} // namespace tca

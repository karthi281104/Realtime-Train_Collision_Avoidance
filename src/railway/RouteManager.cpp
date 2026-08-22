#include "railway/RouteManager.hpp"
#include "core/Logger.hpp"
#include <iostream>
#include <sstream>

namespace tca {

TrackId Route::currentTrack() const {
    if(currentSegment < static_cast<int>(trackPath.size()))
        return trackPath[static_cast<std::size_t>(currentSegment)];
    return 0;
}
TrackId Route::nextTrack() const {
    std::size_t next = static_cast<std::size_t>(currentSegment)+1;
    if(next < trackPath.size()) return trackPath[next];
    return 0;
}

RouteManager::RouteManager(RailwayNetwork& net) : net_(net) {}

RouteId RouteManager::assignRoute(TrainId tid, uint32_t src, uint32_t dst) {
    auto nodePath = net_.dijkstra(src, dst);
    if(nodePath.empty()) {
        LOG_WARN("Route: no path from " << src << " to " << dst);
        return 0;
    }
    auto trackPath = net_.nodePathToTracks(nodePath);

    Route r;
    r.id = nextId_++;
    r.trainId = tid;
    r.srcNode = src;
    r.dstNode = dst;
    r.nodePath = std::move(nodePath);
    r.trackPath = std::move(trackPath);
    r.currentSegment = 0;

    RouteId rid = r.id;
    routes_[rid] = std::move(r);
    trainRoute_[tid] = rid;
    return rid;
}

Route* RouteManager::route(RouteId id) {
    auto it = routes_.find(id); return it!=routes_.end() ? &it->second : nullptr;
}
const Route* RouteManager::route(RouteId id) const {
    auto it = routes_.find(id); return it!=routes_.end() ? &it->second : nullptr;
}
Route* RouteManager::routeForTrain(TrainId tid) {
    auto it = trainRoute_.find(tid);
    if(it == trainRoute_.end()) return nullptr;
    return route(it->second);
}
void RouteManager::removeRoute(RouteId id) {
    if(auto* r = route(id)) trainRoute_.erase(r->trainId);
    routes_.erase(id);
}
void RouteManager::print() const {
    std::cout << "\n═══ ROUTES ═══\n";
    for(auto& [id,r] : routes_) {
        std::cout << "  Route#" << id << " Train#" << r.trainId << ": ";
        for(auto n : r.nodePath) {
            if(auto* nd = net_.node(n)) std::cout << nd->name << " ";
        }
        std::cout << "\n";
    }
}

} // namespace tca

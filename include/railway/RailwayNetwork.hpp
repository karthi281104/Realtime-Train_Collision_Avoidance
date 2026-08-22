#pragma once
#include "railway/Track.hpp"
#include "railway/Station.hpp"
#include "railway/Junction.hpp"
#include "railway/Signal.hpp"
#include <optional>
#include <unordered_map>
#include <vector>

namespace tca {

/// Adjacency list entry.
struct Edge {
    TrackId   trackId{0};
    uint32_t  toNode{0};
    double    weight{0.0};   // metres (Dijkstra weight)
};

/// Node in the graph – may represent a Station or Junction.
struct NetworkNode {
    uint32_t    id{0};
    std::string name;
    bool        isStation{false};
    bool        isJunction{false};
    std::vector<Edge> edges;
};

class RailwayNetwork {
public:
    RailwayNetwork() = default;

    // ── Builder ──────────────────────────────────────────────────────────────
    uint32_t addStation (const std::string& name, double x=0, double y=0, int platforms=1);
    uint32_t addJunction(const std::string& name, JunctionType jt = JunctionType::SIMPLE);
    TrackId  addTrack   (const std::string& name,
                         uint32_t fromNode, uint32_t toNode,
                         double lengthM, double speedLimitKmh,
                         bool bidirectional = true);
    SignalId addSignal  (TrackId trackId, double posM);

    // ── Accessors ─────────────────────────────────────────────────────────────
    NetworkNode*  node   (uint32_t id);
    Track*        track  (TrackId  id);
    Station*      station(StationId id);
    Junction*     junction(JunctionId id);
    Signal*       signal  (SignalId id);

    const NetworkNode*  node   (uint32_t id)    const;
    const Track*        track  (TrackId  id)    const;

    std::vector<uint32_t>       neighbours(uint32_t nodeId) const;
    std::vector<TrackId>        tracksAt  (uint32_t nodeId) const;

    // ── Graph algorithms ─────────────────────────────────────────────────────
    std::vector<uint32_t> bfs  (uint32_t src, uint32_t dst) const;
    std::vector<uint32_t> dfs  (uint32_t src, uint32_t dst) const;
    /// Returns node path; empty if unreachable.
    std::vector<uint32_t> dijkstra(uint32_t src, uint32_t dst) const;
    /// Returns track sequence for a node path.
    std::vector<TrackId>  nodePathToTracks(const std::vector<uint32_t>& nodes) const;

    // ── Occupancy ─────────────────────────────────────────────────────────────
    void trainEntersTrack(TrainId tid, TrackId  tkId);
    void trainLeavesTrack(TrainId tid, TrackId  tkId);

    // ── Disable/enable ────────────────────────────────────────────────────────
    void disableTrack(TrackId id);
    void enableTrack (TrackId id);

    void clear() {
        nextNodeId_ = 1;
        nextTrackId_ = 1;
        nextSignalId_ = 1;
        nodes_.clear();
        tracks_.clear();
        stations_.clear();
        junctions_.clear();
        signals_.clear();
    }

    void print() const;

private:
    uint32_t nextNodeId_{1};
    TrackId  nextTrackId_{1};
    SignalId nextSignalId_{1};

    std::unordered_map<uint32_t,  NetworkNode> nodes_;
    std::unordered_map<TrackId,   Track>       tracks_;
    std::unordered_map<StationId, Station>     stations_;
    std::unordered_map<JunctionId,Junction>    junctions_;
    std::unordered_map<SignalId,  Signal>      signals_;

    TrackId findTrack(uint32_t from, uint32_t to) const;
};

} // namespace tca

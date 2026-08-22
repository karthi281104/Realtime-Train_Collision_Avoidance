#include "railway/RailwayNetwork.hpp"
#include "core/Logger.hpp"
#include <algorithm>
#include <cassert>
#include <format>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace tca {

// ─── Builder ──────────────────────────────────────────────────────────────────

uint32_t RailwayNetwork::addStation(const std::string& name, double x, double y, int platforms) {
    uint32_t id = nextNodeId_++;
    NetworkNode nd;
    nd.id = id; nd.name = name; nd.isStation = true;
    nodes_[id] = std::move(nd);

    Station st;
    st.id = static_cast<StationId>(id);
    st.name = name; st.posX = x; st.posY = y; st.platform = platforms;
    stations_[st.id] = std::move(st);
    return id;
}

uint32_t RailwayNetwork::addJunction(const std::string& name, JunctionType jt) {
    uint32_t id = nextNodeId_++;
    NetworkNode nd;
    nd.id = id; nd.name = name; nd.isJunction = true;
    nodes_[id] = std::move(nd);

    Junction jn;
    jn.id = static_cast<JunctionId>(id); jn.name = name; jn.type = jt;
    junctions_[jn.id] = std::move(jn);
    return id;
}

TrackId RailwayNetwork::addTrack(const std::string& name,
                                  uint32_t from, uint32_t to,
                                  double lengthM, double speedLimitKmh,
                                  bool bidir) {
    TrackId tid = nextTrackId_++;
    Track tk;
    tk.id = tid; tk.name = name;
    tk.fromNode = from; tk.toNode = to;
    tk.lengthM = lengthM;
    tk.speedLimitMs = speedLimitKmh * kMsToMs;
    tk.bidirectional = bidir;
    tracks_[tid] = std::move(tk);

    // Add edges
    Edge e1{tid, to,   lengthM};
    nodes_[from].edges.push_back(e1);
    if(bidir) {
        Edge e2{tid, from, lengthM};
        nodes_[to].edges.push_back(e2);
    }

    // Record in junction if applicable
    if(nodes_[from].isJunction) junctions_[from].connectedTracks.push_back(tid);
    if(nodes_[to].isJunction)   junctions_[to  ].connectedTracks.push_back(tid);

    return tid;
}

SignalId RailwayNetwork::addSignal(TrackId tkId, double posM) {
    SignalId sid = nextSignalId_++;
    Signal sg;
    sg.id = sid; sg.trackId = tkId; sg.posOnTrackM = posM;
    signals_[sid] = std::move(sg);
    return sid;
}

// ─── Accessors ────────────────────────────────────────────────────────────────

NetworkNode* RailwayNetwork::node(uint32_t id) {
    auto it = nodes_.find(id); return it!=nodes_.end() ? &it->second : nullptr;
}
const NetworkNode* RailwayNetwork::node(uint32_t id) const {
    auto it = nodes_.find(id); return it!=nodes_.end() ? &it->second : nullptr;
}
Track*  RailwayNetwork::track(TrackId id) {
    auto it = tracks_.find(id); return it!=tracks_.end() ? &it->second : nullptr;
}
const Track*  RailwayNetwork::track(TrackId id) const {
    auto it = tracks_.find(id); return it!=tracks_.end() ? &it->second : nullptr;
}
Station*  RailwayNetwork::station(StationId id) {
    auto it = stations_.find(id); return it!=stations_.end() ? &it->second : nullptr;
}
Junction* RailwayNetwork::junction(JunctionId id) {
    auto it = junctions_.find(id); return it!=junctions_.end() ? &it->second : nullptr;
}
Signal*   RailwayNetwork::signal(SignalId id) {
    auto it = signals_.find(id); return it!=signals_.end() ? &it->second : nullptr;
}

std::vector<uint32_t> RailwayNetwork::neighbours(uint32_t nodeId) const {
    std::vector<uint32_t> res;
    if(auto* nd = node(nodeId))
        for(auto& e : nd->edges) {
            auto* tk = track(e.trackId);
            if(tk && tk->enabled) res.push_back(e.toNode);
        }
    return res;
}

std::vector<TrackId> RailwayNetwork::tracksAt(uint32_t nodeId) const {
    std::vector<TrackId> res;
    if(auto* nd = node(nodeId))
        for(auto& e : nd->edges) res.push_back(e.trackId);
    return res;
}

// ─── Graph algorithms ────────────────────────────────────────────────────────

std::vector<uint32_t> RailwayNetwork::bfs(uint32_t src, uint32_t dst) const {
    std::unordered_map<uint32_t,uint32_t> parent;
    std::queue<uint32_t> q;
    parent[src] = src;
    q.push(src);
    while(!q.empty()) {
        auto cur = q.front(); q.pop();
        if(cur == dst) break;
        for(auto nb : neighbours(cur))
            if(!parent.count(nb)) { parent[nb]=cur; q.push(nb); }
    }
    if(!parent.count(dst)) return {};
    std::vector<uint32_t> path;
    for(auto n=dst; n!=src; n=parent[n]) path.push_back(n);
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<uint32_t> RailwayNetwork::dfs(uint32_t src, uint32_t dst) const {
    std::unordered_map<uint32_t,uint32_t> parent;
    std::vector<uint32_t> stack{src};
    parent[src] = src;
    while(!stack.empty()) {
        auto cur = stack.back(); stack.pop_back();
        if(cur == dst) break;
        for(auto nb : neighbours(cur))
            if(!parent.count(nb)) { parent[nb]=cur; stack.push_back(nb); }
    }
    if(!parent.count(dst)) return {};
    std::vector<uint32_t> path;
    for(auto n=dst; n!=src; n=parent[n]) path.push_back(n);
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<uint32_t> RailwayNetwork::dijkstra(uint32_t src, uint32_t dst) const {
    using P = std::pair<double,uint32_t>;
    std::unordered_map<uint32_t,double>   dist;
    std::unordered_map<uint32_t,uint32_t> prev;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    for(auto& [id,_] : nodes_) dist[id] = kInfinity;
    dist[src] = 0.0;
    pq.push({0.0, src});

    while(!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if(d > dist[u]) continue;
        if(u == dst) break;
        auto* nd = node(u);
        if(!nd) continue;
        for(auto& e : nd->edges) {
            auto* tk = track(e.trackId);
            if(!tk || !tk->enabled) continue;
            double nd2 = d + e.weight;
            if(nd2 < dist[e.toNode]) {
                dist[e.toNode] = nd2;
                prev[e.toNode] = u;
                pq.push({nd2, e.toNode});
            }
        }
    }
    if(dist[dst] == kInfinity) return {};
    std::vector<uint32_t> path;
    for(auto n=dst; n!=src; n=prev[n]) path.push_back(n);
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<TrackId> RailwayNetwork::nodePathToTracks(const std::vector<uint32_t>& nodes) const {
    std::vector<TrackId> res;
    for(std::size_t i=0; i+1<nodes.size(); ++i)
        res.push_back(findTrack(nodes[i], nodes[i+1]));
    return res;
}

TrackId RailwayNetwork::findTrack(uint32_t from, uint32_t to) const {
    auto* nd = node(from);
    if(!nd) return 0;
    for(auto& e : nd->edges)
        if(e.toNode == to) return e.trackId;
    return 0;
}

// ─── Occupancy ────────────────────────────────────────────────────────────────

void RailwayNetwork::trainEntersTrack(TrainId tid, TrackId tkId) {
    if(auto* tk = track(tkId)) tk->addOccupant(tid);
}
void RailwayNetwork::trainLeavesTrack(TrainId tid, TrackId tkId) {
    if(auto* tk = track(tkId)) tk->removeOccupant(tid);
}

void RailwayNetwork::disableTrack(TrackId id) {
    if(auto* tk = track(id)) tk->enabled = false;
}
void RailwayNetwork::enableTrack(TrackId id) {
    if(auto* tk = track(id)) tk->enabled = true;
}

void RailwayNetwork::print() const {
    std::cout << "\n═══ RAILWAY NETWORK ═══\n";
    std::cout << "Nodes: " << nodes_.size()
              << "  Tracks: " << tracks_.size()
              << "  Signals: " << signals_.size() << "\n";
    for(auto& [id, nd] : nodes_) {
        std::cout << "  [" << nd.name << "]";
        if(nd.isStation)  std::cout << " (Station)";
        if(nd.isJunction) std::cout << " (Junction)";
        std::cout << " → ";
        for(auto& e : nd.edges)
            if(auto* tk = track(e.trackId))
                std::cout << nd.name << "->" << node(e.toNode)->name
                          << "(" << static_cast<int>(tk->lengthM) << "m) ";
        std::cout << "\n";
    }
    std::cout << "\n";
}

} // namespace tca

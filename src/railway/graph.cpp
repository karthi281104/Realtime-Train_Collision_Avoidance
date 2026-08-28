#include "graph.h"
#include <algorithm>
#include <cmath>

namespace railway {

int Graph::addNode(const std::string& name, double latitude, double longitude) {
    int node_id = node_counter++;
    nodes.emplace(node_id, Node(node_id, name, latitude, longitude));
    return node_id;
}

bool Graph::addTrack(int from_id, int to_id, double distance, double speed_limit) {
    if (!nodeExists(from_id) || !nodeExists(to_id)) {
        return false;
    }
    
    // Add bidirectional connection
    nodes[from_id].edges.emplace_back(to_id, distance, speed_limit);
    nodes[to_id].edges.emplace_back(from_id, distance, speed_limit);
    return true;
}

bool Graph::addDirectedTrack(int from_id, int to_id, double distance, double speed_limit) {
    if (!nodeExists(from_id) || !nodeExists(to_id)) {
        return false;
    }
    
    // Add unidirectional connection
    nodes[from_id].edges.emplace_back(to_id, distance, speed_limit);
    return true;
}

bool Graph::setTrackOperational(int from_id, int to_id, bool operational) {
    if (!nodeExists(from_id)) return false;
    
    auto& edges = nodes[from_id].edges;
    auto it = std::find_if(edges.begin(), edges.end(),
        [to_id](const Edge& e) { return e.target_node_id == to_id; });
    
    if (it != edges.end()) {
        it->is_operational = operational;
        return true;
    }
    return false;
}

std::vector<int> Graph::findShortestPath(int start_id, int end_id) {
    if (!nodeExists(start_id) || !nodeExists(end_id)) {
        return {};
    }
    
    // Dijkstra's algorithm
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> previous;
    std::vector<bool> visited(node_counter, false);
    
    // Initialize
    for (const auto& [id, _] : nodes) {
        distances[id] = std::numeric_limits<double>::max();
    }
    distances[start_id] = 0.0;
    
    for (int i = 0; i < node_counter; ++i) {
        int current = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        // Find unvisited node with minimum distance
        for (const auto& [id, _] : nodes) {
            if (!visited[id] && distances[id] < min_dist) {
                current = id;
                min_dist = distances[id];
            }
        }
        
        if (current == -1 || current == end_id) break;
        visited[current] = true;
        
        // Update distances to neighbors
        for (const auto& edge : nodes[current].edges) {
            if (!edge.is_operational) continue;
            
            int neighbor = edge.target_node_id;
            double new_dist = distances[current] + edge.distance_meters;
            
            if (new_dist < distances[neighbor]) {
                distances[neighbor] = new_dist;
                previous[neighbor] = current;
            }
        }
    }
    
    // Reconstruct path
    std::vector<int> path;
    int current = end_id;
    while (current != -1) {
        path.insert(path.begin(), current);
        if (current == start_id) break;
        current = previous.count(current) ? previous[current] : -1;
    }
    
    return path;
}

std::vector<int> Graph::getReachableNodes(int start_id) {
    if (!nodeExists(start_id)) {
        return {};
    }
    
    std::vector<int> reachable;
    std::vector<bool> visited(node_counter, false);
    std::queue<int> q;
    
    q.push(start_id);
    visited[start_id] = true;
    
    while (!q.empty()) {
        int current = q.front();
        q.pop();
        reachable.push_back(current);
        
        for (const auto& edge : nodes[current].edges) {
            if (!edge.is_operational) continue;
            if (!visited[edge.target_node_id]) {
                visited[edge.target_node_id] = true;
                q.push(edge.target_node_id);
            }
        }
    }
    
    return reachable;
}

std::vector<int> Graph::getAdjacentNodes(int node_id) {
    if (!nodeExists(node_id)) {
        return {};
    }
    
    std::vector<int> adjacent;
    for (const auto& edge : nodes[node_id].edges) {
        if (edge.is_operational) {
            adjacent.push_back(edge.target_node_id);
        }
    }
    return adjacent;
}

double Graph::getTrackDistance(int from_id, int to_id) {
    if (!nodeExists(from_id)) return -1.0;
    
    for (const auto& edge : nodes[from_id].edges) {
        if (edge.target_node_id == to_id && edge.is_operational) {
            return edge.distance_meters;
        }
    }
    return -1.0;
}

double Graph::getSpeedLimit(int from_id, int to_id) {
    if (!nodeExists(from_id)) return -1.0;
    
    for (const auto& edge : nodes[from_id].edges) {
        if (edge.target_node_id == to_id && edge.is_operational) {
            return edge.speed_limit_ms;
        }
    }
    return -1.0;
}

const Node& Graph::getNode(int node_id) const {
    return nodes.at(node_id);
}

bool Graph::nodeExists(int node_id) const {
    return nodes.find(node_id) != nodes.end();
}

int Graph::getEdgeCount() const {
    int count = 0;
    for (const auto& [_, node] : nodes) {
        count += node.edges.size();
    }
    return count;
}

void Graph::clear() {
    nodes.clear();
    node_counter = 0;
}

double Graph::calculateGPSDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; // Earth radius in km
    
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double delta_lat = (lat2 - lat1) * M_PI / 180.0;
    double delta_lon = (lon2 - lon1) * M_PI / 180.0;
    
    double a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
    
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}

} // namespace railway

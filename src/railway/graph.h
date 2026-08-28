#ifndef RAILWAY_GRAPH_H
#define RAILWAY_GRAPH_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <string>
#include <queue>
#include <limits>
#include <cmath>

namespace railway {

/**
 * @struct Edge
 * @brief Represents a track connection between two nodes
 */
struct Edge {
    int target_node_id;      // ID of destination node
    double distance_meters;  // Track length in meters
    double speed_limit_ms;   // Maximum speed on this track (m/s)
    bool is_operational;     // Is track operational?
    
    Edge(int target, double dist, double speed, bool operational = true)
        : target_node_id(target), distance_meters(dist), speed_limit_ms(speed),
          is_operational(operational) {}
};

/**
 * @struct Node
 * @brief Represents a station or track junction
 */
struct Node {
    int node_id;
    std::string name;
    double latitude;         // GPS coordinates
    double longitude;
    std::vector<Edge> edges; // Outgoing connections
    
    Node(int id, const std::string& node_name, double lat, double lon)
        : node_id(id), name(node_name), latitude(lat), longitude(lon) {}
};

/**
 * @class Graph
 * @brief Railway network topology representation using adjacency list
 * 
 * WEEK 1 DELIVERABLE: Fully Implemented
 * - Track network representation
 * - Shortest path algorithms (Dijkstra)
 * - Graph traversal & query operations
 */
class Graph {
private:
    std::unordered_map<int, Node> nodes;  // All stations/junctions
    int node_counter = 0;
    
public:
    /**
     * @brief Add a station/junction node to the graph
     * @param name Station name
     * @param latitude GPS latitude
     * @param longitude GPS longitude
     * @return Node ID of created node
     */
    int addNode(const std::string& name, double latitude, double longitude);
    
    /**
     * @brief Add a bidirectional track connection
     * @param from_id Source node ID
     * @param to_id Destination node ID
     * @param distance Track length in meters
     * @param speed_limit Maximum speed (m/s)
     * @return true if successful
     */
    bool addTrack(int from_id, int to_id, double distance, double speed_limit);
    
    /**
     * @brief Add unidirectional track connection
     * @param from_id Source node ID
     * @param to_id Destination node ID
     * @param distance Track length in meters
     * @param speed_limit Maximum speed (m/s)
     * @return true if successful
     */
    bool addDirectedTrack(int from_id, int to_id, double distance, double speed_limit);
    
    /**
     * @brief Set track operational status
     * @param from_id Source node
     * @param to_id Destination node
     * @param operational Operational status
     * @return true if track found
     */
    bool setTrackOperational(int from_id, int to_id, bool operational);
    
    /**
     * @brief Find shortest path using Dijkstra algorithm
     * @param start_id Starting node ID
     * @param end_id Destination node ID
     * @return Vector of node IDs representing shortest path
     */
    std::vector<int> findShortestPath(int start_id, int end_id);
    
    /**
     * @brief Find all reachable nodes from a starting point
     * @param start_id Starting node ID
     * @return Vector of reachable node IDs
     */
    std::vector<int> getReachableNodes(int start_id);
    
    /**
     * @brief Get all adjacent nodes
     * @param node_id Query node ID
     * @return Vector of adjacent node IDs
     */
    std::vector<int> getAdjacentNodes(int node_id);
    
    /**
     * @brief Get distance between two directly connected nodes
     * @param from_id Source node
     * @param to_id Destination node
     * @return Distance in meters, or -1 if not connected
     */
    double getTrackDistance(int from_id, int to_id);
    
    /**
     * @brief Get speed limit on track
     * @param from_id Source node
     * @param to_id Destination node
     * @return Speed limit in m/s, or -1 if not connected
     */
    double getSpeedLimit(int from_id, int to_id);
    
    /**
     * @brief Get node information
     * @param node_id Query node ID
     * @return const reference to Node
     */
    const Node& getNode(int node_id) const;
    
    /**
     * @brief Check if node exists
     * @param node_id Query node ID
     * @return true if node exists
     */
    bool nodeExists(int node_id) const;
    
    /**
     * @brief Get total number of nodes
     * @return Node count
     */
    int getNodeCount() const { return nodes.size(); }
    
    /**
     * @brief Get total number of edges
     * @return Edge count
     */
    int getEdgeCount() const;
    
    /**
     * @brief Clear all nodes and edges
     */
    void clear();
    
    /**
     * @brief Calculate Euclidean distance between two GPS coordinates
     * @param lat1 Latitude of point 1
     * @param lon1 Longitude of point 1
     * @param lat2 Latitude of point 2
     * @param lon2 Longitude of point 2
     * @return Distance in kilometers
     */
    static double calculateGPSDistance(double lat1, double lon1, double lat2, double lon2);
};

} // namespace railway

#endif // RAILWAY_GRAPH_H

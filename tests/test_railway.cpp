/// test_railway.cpp — Unit tests for the railway network graph.

#include "railway/RailwayNetwork.hpp"
#include "railway/RouteManager.hpp"
#include <cassert>
#include <iostream>

using namespace tca;

static int gPass = 0, gFail = 0;

#define CHECK(cond, msg) do { \
    if(cond) { ++gPass; std::cout << "  PASS: " << msg << "\n"; } \
    else     { ++gFail; std::cout << "  FAIL: " << msg << "\n"; } \
} while(0)

void test_basic_graph() {
    std::cout << "\n[Basic Graph Construction]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    auto t1 = net.addTrack("A-B", A, B, 1000.0, 120.0);
    auto t2 = net.addTrack("B-C", B, C, 2000.0, 160.0);

    CHECK(net.node(A) != nullptr, "station A exists");
    CHECK(net.track(t1) != nullptr, "track A-B exists");
    CHECK(net.track(t1)->lengthM == 1000.0, "track A-B length = 1000m");
    CHECK(net.track(t2)->speedLimitMs > 0, "track B-C has speed limit");
    CHECK(net.neighbours(B).size() == 2, "B has 2 neighbours (bidirectional)");
}

void test_bfs() {
    std::cout << "\n[BFS]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    auto D = net.addStation("D");
    net.addTrack("A-B", A, B, 500, 100);
    net.addTrack("B-C", B, C, 500, 100);
    net.addTrack("C-D", C, D, 500, 100);
    net.addTrack("A-D", A, D, 3000, 100);  // longer direct path

    auto path = net.bfs(A, D);
    CHECK(!path.empty(), "BFS finds path A→D");
    CHECK(path.front() == A && path.back() == D, "BFS path starts at A, ends at D");
}

void test_dijkstra_shortest() {
    std::cout << "\n[Dijkstra Shortest Path]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    auto D = net.addStation("D");
    net.addTrack("A-B", A, B, 1000, 100);
    net.addTrack("B-D", B, D, 1000, 100);
    net.addTrack("A-C", A, C, 500,  100);
    net.addTrack("C-D", C, D, 500,  100);  // shorter route: A→C→D=1000m vs A→B→D=2000m

    auto path = net.dijkstra(A, D);
    CHECK(!path.empty(), "Dijkstra finds path A→D");
    // Shortest: A→C→D (1000m)
    CHECK(path.size() == 3, "Dijkstra picks A→C→D (3 nodes)");
    if(path.size() == 3)
        CHECK(path[1] == C, "Dijkstra via C");
}

void test_unreachable() {
    std::cout << "\n[Unreachable Node]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    net.addTrack("A-B", A, B, 500, 100, false);  // one-way A→B
    // C is isolated

    auto p = net.dijkstra(A, C);
    CHECK(p.empty(), "No path A→C (C isolated)");
}

void test_disabled_track() {
    std::cout << "\n[Disabled Track]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    auto t1 = net.addTrack("A-B", A, B, 500, 100);
    net.addTrack("B-C", B, C, 500, 100);
    net.addTrack("A-C", A, C, 2000, 100);

    // Disable A-B; shortest path must go A→C direct
    net.disableTrack(t1);
    auto path = net.dijkstra(A, C);
    CHECK(!path.empty(), "Path exists after disabling A-B");
    // With A-B disabled, direct A-C used
    bool usedDirect = (path.size() == 2);
    CHECK(usedDirect, "Dijkstra uses direct A-C when A-B disabled");
}

void test_occupancy() {
    std::cout << "\n[Occupancy]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto t1 = net.addTrack("A-B", A, B, 1000, 100);

    net.trainEntersTrack(1, t1);
    net.trainEntersTrack(2, t1);
    CHECK(net.track(t1)->isOccupied(), "Track occupied by 2 trains");
    net.trainLeavesTrack(1, t1);
    CHECK(net.track(t1)->isOccupied(), "Track still occupied (1 train)");
    net.trainLeavesTrack(2, t1);
    CHECK(!net.track(t1)->isOccupied(), "Track free after both leave");
}

void test_route_manager() {
    std::cout << "\n[Route Manager]\n";
    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto C = net.addStation("C");
    net.addTrack("A-B", A, B, 1000, 100);
    net.addTrack("B-C", B, C, 1000, 100);

    RouteManager rm(net);
    auto rid = rm.assignRoute(1, A, C);
    CHECK(rid > 0, "Route assigned");
    auto* r = rm.route(rid);
    CHECK(r != nullptr, "Route retrievable");
    CHECK(r->trackPath.size() == 2, "Route A→B→C has 2 track segments");
    CHECK(!r->isComplete(), "Route not complete at start");
}

int main() {
    std::cout << "════════════════════════════════════════\n";
    std::cout << "  RAILWAY NETWORK UNIT TESTS\n";
    std::cout << "════════════════════════════════════════\n";

    test_basic_graph();
    test_bfs();
    test_dijkstra_shortest();
    test_unreachable();
    test_disabled_track();
    test_occupancy();
    test_route_manager();

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "  Results: " << gPass << " PASSED, " << gFail << " FAILED\n";
    std::cout << "════════════════════════════════════════\n";
    return gFail > 0 ? 1 : 0;
}

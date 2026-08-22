/// test_physics.cpp — Unit tests for kinematics and TTC calculations.
/// No external framework: simple assertion macros.

#include "train/TrainPhysics.hpp"
#include "train/TrainTypes.hpp"
#include <cmath>
#include <iostream>
#include <string>

using namespace tca;
using namespace tca::physics;

// ─── Minimal test harness ─────────────────────────────────────────────────────
static int gPass = 0, gFail = 0;

#define CHECK(cond, msg) do { \
    if(cond) { ++gPass; std::cout << "  PASS: " << msg << "\n"; } \
    else     { ++gFail; std::cout << "  FAIL: " << msg << "\n"; } \
} while(0)

#define CHECK_NEAR(a, b, tol, msg) \
    CHECK(std::abs((a)-(b)) <= (tol), \
          msg << " [got=" << (a) << " expected≈" << (b) << " tol=" << (tol) << "]")

// ─── Tests ────────────────────────────────────────────────────────────────────

void test_euler_integration() {
    std::cout << "\n[Euler Integration]\n";
    double pos = 0.0, vel = 10.0, accel = 2.0, dt = 0.05;
    integrate(pos, vel, accel, dt);
    // x = 0 + 10*0.05 + 0.5*2*0.05² = 0.5 + 0.0025 = 0.5025
    // v = 10 + 2*0.05 = 10.1
    CHECK_NEAR(pos, 0.5025, 1e-6, "pos after 1 tick");
    CHECK_NEAR(vel, 10.1,   1e-6, "vel after 1 tick");
}

void test_negative_speed_clamp() {
    std::cout << "\n[Negative Speed Clamp]\n";
    double pos = 100.0, vel = 0.5, accel = -5.0, dt = 1.0;
    integrate(pos, vel, accel, dt);
    CHECK(vel >= 0.0, "velocity cannot go negative");
}

void test_braking_distance() {
    std::cout << "\n[Braking Distance]\n";
    // v=20 m/s, decel=1 m/s², reaction=1.5s
    // d_r = 20*1.5 = 30m, d_b = 20²/(2*1) = 200m, total = 230m
    double bd = brakingDistance(20.0, 1.0, 1.5);
    CHECK_NEAR(bd, 230.0, 1.0, "braking distance at 20m/s");

    // v=0: should be 0
    CHECK_NEAR(brakingDistance(0.0, 1.0, 1.5), 0.0, 1e-9, "braking dist at v=0");
}

void test_ttc_same_direction() {
    std::cout << "\n[TTC Same Direction]\n";
    // Leading at pos=500, vel=10;  Trailing at pos=0, vel=20; safeGap=100m
    // relVel=10, gap=500, gapToClose=400, TTC=40s
    double ttc = ttcSameDirection(500.0, 10.0, 0.0, 20.0, 100.0);
    CHECK_NEAR(ttc, 40.0, 0.5, "TTC rear-end (expected 40s)");

    // Already safe (trailing slower than leading)
    double ttcSafe = ttcSameDirection(500.0, 20.0, 0.0, 10.0, 100.0);
    CHECK(ttcSafe >= 1e10, "TTC infinite when diverging");
}

void test_ttc_head_on() {
    std::cout << "\n[TTC Head-On]\n";
    // posA=0, velA=20;  posB=2000, velB=20  → gap=2000, closing=40 m/s → TTC=50s
    double ttc = ttcHeadOn(0.0, 20.0, 2000.0, 20.0);
    CHECK_NEAR(ttc, 50.0, 0.5, "TTC head-on (expected 50s)");
}

void test_required_separation() {
    std::cout << "\n[Required Separation]\n";
    TrainData td;
    td.spec = TrainSpec::forPassenger();
    td.velocityMs = 160.0 / 3.6;  // 160 km/h
    double rs = requiredSeparation(td);
    // Should be > length (180m) + some braking distance + margin
    CHECK(rs > 180.0, "required separation > train length");
    CHECK(rs < 5000.0, "required separation < 5km (sanity)");
    std::cout << "    Required separation at 160km/h: " << static_cast<int>(rs) << "m\n";
}

void test_multi_tick_convergence() {
    std::cout << "\n[Multi-tick Convergence]\n";
    // Simulate 10 seconds of a train braking from 80 km/h to 0
    double vel = 80.0 / 3.6;   // m/s
    double pos = 0.0;
    double decel = 1.1;         // m/s²
    double dt    = 0.05;
    int    ticks = 0;

    while(vel > 0.01 && ticks < 10000) {
        double accel = -decel;
        integrate(pos, vel, accel, dt);
        ++ticks;
    }
    double simTime = ticks * dt;
    // v/a = 22.2/1.1 ≈ 20.2s
    CHECK_NEAR(simTime, 20.2, 1.5, "time to stop from 80km/h");
    std::cout << "    Stopped in " << static_cast<int>(simTime) << "s, pos=" << static_cast<int>(pos) << "m\n";
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "════════════════════════════════════════\n";
    std::cout << "  PHYSICS UNIT TESTS\n";
    std::cout << "════════════════════════════════════════\n";

    test_euler_integration();
    test_negative_speed_clamp();
    test_braking_distance();
    test_ttc_same_direction();
    test_ttc_head_on();
    test_required_separation();
    test_multi_tick_convergence();

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "  Results: " << gPass << " PASSED, " << gFail << " FAILED\n";
    std::cout << "════════════════════════════════════════\n";
    return gFail > 0 ? 1 : 0;
}

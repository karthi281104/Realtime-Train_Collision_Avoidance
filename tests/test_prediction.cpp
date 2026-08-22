/// test_prediction.cpp — Integration tests for collision prediction + resolution.

#include "railway/RailwayNetwork.hpp"
#include "railway/RouteManager.hpp"
#include "train/TrainSubtypes.hpp"
#include "train/TrainStateManager.hpp"
#include "train/TrainPhysics.hpp"
#include "prediction/CollisionPredictor.hpp"
#include "safety/ConflictManager.hpp"
#include "safety/ConflictResolver.hpp"
#include "core/Logger.hpp"
#include <cmath>
#include <iostream>
#include <memory>

using namespace tca;

static int gPass = 0, gFail = 0;

#define CHECK(cond, msg) do { \
    if(cond) { ++gPass; std::cout << "  PASS: " << msg << "\n"; } \
    else     { ++gFail; std::cout << "  FAIL: " << msg << "\n"; } \
} while(0)

#define CHECK_NEAR(a, b, tol, msg) \
    CHECK(std::abs((a)-(b)) <= (tol), \
          msg << " [got=" << (a) << " exp≈" << (b) << "]")

// ─── Test 1: Rear-end conflict detection ──────────────────────────────────────
void test_rear_end_detection() {
    std::cout << "\n[Rear-End Detection]\n";

    RailwayNetwork net;
    auto A = net.addStation("A");
    auto B = net.addStation("B");
    auto tk = net.addTrack("A-B", A, B, 10000.0, 160.0);

    TrainData leading, trailing;
    leading.id = 1; leading.type = TrainType::PASSENGER;
    leading.spec = TrainSpec::forPassenger();
    leading.positionM = 800.0; leading.velocityMs = 40.0 / 3.6; // 40 km/h
    leading.currentTrackId = tk; leading.direction = Direction::FORWARD;
    leading.state = TrainState::RUNNING;

    trailing.id = 2; trailing.type = TrainType::EXPRESS;
    trailing.spec = TrainSpec::forExpress();
    trailing.positionM = 0.0; trailing.velocityMs = 80.0 / 3.6; // 80 km/h
    trailing.currentTrackId = tk; trailing.direction = Direction::FORWARD;
    trailing.state = TrainState::RUNNING;
    trailing.targetSpeedMs = trailing.velocityMs;
    leading.targetSpeedMs  = leading.velocityMs;

    CollisionPredictor pred(net, 30.0, 0.05);
    auto pr = pred.predict(leading, trailing, 0.0);

    CHECK(pr.hasConflict, "Rear-end conflict detected at 500m separation, 40m/s relative");
    if(pr.hasConflict) {
        CHECK(pr.conflict.type == ConflictType::REAR_END, "Conflict type = REAR_END");
        CHECK(pr.conflict.ttcSeconds < 30.0, "TTC < 30s");
        CHECK(pr.conflict.risk >= RiskLevel::CAUTION, "Risk >= CAUTION");
        std::cout << "    TTC=" << pr.conflict.ttcSeconds
                  << "s  Sep=" << static_cast<int>(pr.conflict.separationM) << "m"
                  << "  Risk=" << toString(pr.conflict.risk) << "\n";
    }
}

// ─── Test 2: No conflict when separation is large ─────────────────────────────
void test_no_conflict_large_gap() {
    std::cout << "\n[No Conflict When Safe]\n";

    RailwayNetwork net;
    auto A = net.addStation("A"); auto B = net.addStation("B");
    auto tk = net.addTrack("A-B", A, B, 50000.0, 160.0);

    TrainData leading, trailing;
    leading.id = 1; leading.spec = TrainSpec::forPassenger();
    leading.positionM = 20000.0; leading.velocityMs = 80.0 / 3.6;
    leading.currentTrackId = tk; leading.direction = Direction::FORWARD;
    leading.state = TrainState::RUNNING; leading.targetSpeedMs = leading.velocityMs;

    trailing.id = 2; trailing.spec = TrainSpec::forPassenger();
    trailing.positionM = 0.0; trailing.velocityMs = 80.0 / 3.6;  // same speed
    trailing.currentTrackId = tk; trailing.direction = Direction::FORWARD;
    trailing.state = TrainState::RUNNING; trailing.targetSpeedMs = trailing.velocityMs;

    CollisionPredictor pred(net, 30.0, 0.05);
    auto pr = pred.predict(leading, trailing, 0.0);

    CHECK(!pr.hasConflict, "No conflict when 20km apart at same speed");
}

// ─── Test 3: TTC mathematical validation ─────────────────────────────────────
void test_ttc_math() {
    std::cout << "\n[TTC Mathematical Validation]\n";
    // Gap = 500m, trailing vel = 20m/s, leading vel = 10m/s → relVel = 10m/s
    // Safe gap = 100m → time to close = (500-100)/10 = 40s
    double ttc = tca::physics::ttcSameDirection(500.0, 10.0, 0.0, 20.0, 100.0);
    CHECK_NEAR(ttc, 40.0, 1.0, "TTC exact calculation");
}

// ─── Test 4: Closed-loop control (speed reduction resolves conflict) ──────────
void test_closed_loop_control() {
    std::cout << "\n[Closed-Loop Control: Speed Reduction Resolves Rear-End]\n";

    RailwayNetwork net;
    auto A = net.addStation("A"); auto B = net.addStation("B");
    auto tk = net.addTrack("A-B", A, B, 10000.0, 200.0 * kMsToMs);

    RouteManager      rm(net);
    TrainStateManager tsm;
    ConflictManager   cm(net);
    ConflictResolver  cr;

    // Create trains directly in TSM (no physics engine, manually step)
    auto t1 = std::make_unique<PassengerTrain>(1, "T01", A, B, 800.0, 40.0);
    auto t2 = std::make_unique<ExpressTrain>  (2, "T02", A, B,   0.0, 80.0);
    t1->data().currentTrackId = tk; t1->data().state = TrainState::RUNNING;
    t2->data().currentTrackId = tk; t2->data().state = TrainState::RUNNING;
    t1->data().targetSpeedMs = 40.0 * kMsToMs;
    t2->data().targetSpeedMs = 80.0 * kMsToMs;
    tsm.addTrain(std::move(t1));
    tsm.addTrain(std::move(t2));

    bool conflictEverDetected = false;
    bool conflictResolved     = false;
    double simNow = 0.0;
    const double dt = kDefaultDt;

    for(int tick = 0; tick < 2400 && !conflictResolved; ++tick) {
        simNow += dt;

        // Manual physics step
        tsm.forEach([dt, simNow](Train& t) {
            auto& d = t.data();
            if(d.state == TrainState::IDLE || d.state == TrainState::STOPPED) return;
            d.accelerationMs2 = tca::physics::computeAcceleration(d);
            t.applyPhysics(dt);
            d.simTimestamp = simNow;
        });

        auto snap      = tsm.snapshot();
        auto conflicts = cm.update(snap, simNow);

        if(!conflicts.empty()) {
            conflictEverDetected = true;
            cr.resolve(conflicts, tsm, net, simNow);
        }

        if(conflictEverDetected) {
            auto* t01 = tsm.get(1);
            auto* t02 = tsm.get(2);
            if(t01 && t02 && t02->data().velocityMs <= t01->data().velocityMs * 1.15) {
                conflictResolved = true;
                std::cout << "    Resolved at t=" << static_cast<int>(simNow)
                          << "s  T02=" << static_cast<int>(t02->data().velocityMs*kMsToKmh)
                          << "km/h\n";
            }
        }
    }

    CHECK(conflictEverDetected, "Conflict was detected during simulation");
    CHECK(conflictResolved,     "Conflict resolved (T02 speed reduced)");
}


// ─── Test 5: Head-on emergency braking ───────────────────────────────────────
void test_head_on_emergency() {
    std::cout << "\n[Head-On Emergency Braking]\n";

    RailwayNetwork net;
    auto A = net.addStation("A"); auto B = net.addStation("B");
    auto tk = net.addTrack("A-B", A, B, 10000.0, 160.0);

    TrainData t1, t2;
    t1.id = 1; t1.spec = TrainSpec::forPassenger();
    t1.positionM = 0.0;    t1.velocityMs = 80.0/3.6;
    t1.direction = Direction::FORWARD;
    t1.currentTrackId = tk; t1.state = TrainState::RUNNING;
    t1.targetSpeedMs = t1.velocityMs;

    t2.id = 2; t2.spec = TrainSpec::forPassenger();
    t2.positionM = 800.0; t2.velocityMs = 80.0/3.6;
    t2.direction = Direction::BACKWARD;
    t2.currentTrackId = tk; t2.state = TrainState::RUNNING;
    t2.targetSpeedMs = t2.velocityMs;

    CollisionPredictor pred(net, 30.0, 0.05);
    auto pr = pred.predict(t1, t2, 0.0);

    CHECK(pr.hasConflict, "Head-on conflict detected");
    if(pr.hasConflict) {
        CHECK(pr.conflict.type == ConflictType::HEAD_ON, "Type = HEAD_ON");
        CHECK(pr.conflict.risk >= RiskLevel::CRITICAL, "Risk >= CRITICAL");
        CHECK(pr.conflict.recommendedAction == ConflictAction::EMERGENCY_BRAKING,
              "Recommended action = EMERGENCY_BRAKING");
        std::cout << "    TTC=" << static_cast<int>(pr.conflict.ttcSeconds)
                  << "s  Action=" << toString(pr.conflict.recommendedAction) << "\n";
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main() {
    Logger::instance().init("logs", LogLevel::WARNING);  // suppress noise in tests

    std::cout << "════════════════════════════════════════\n";
    std::cout << "  PREDICTION & SAFETY INTEGRATION TESTS\n";
    std::cout << "════════════════════════════════════════\n";

    test_rear_end_detection();
    test_no_conflict_large_gap();
    test_ttc_math();
    test_closed_loop_control();
    test_head_on_emergency();

    std::cout << "\n════════════════════════════════════════\n";
    std::cout << "  Results: " << gPass << " PASSED, " << gFail << " FAILED\n";
    std::cout << "════════════════════════════════════════\n";
    return gFail > 0 ? 1 : 0;
}

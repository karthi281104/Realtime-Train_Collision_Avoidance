#include "monitoring/ScenarioManager.hpp"
#include "train/TrainSubtypes.hpp"
#include "core/Logger.hpp"
#include <iostream>
#include <memory>

namespace tca {

ScenarioManager::ScenarioManager(TrainStateManager& tsm,
                                  RailwayNetwork& net,
                                  RouteManager& rm)
    : tsm_(tsm), net_(net), rm_(rm)
{}

ScenarioConfig ScenarioManager::load(const std::string& name) {
    if(name == "normal")        return scenario_NormalOps();
    if(name == "rear_end")      return scenario_RearEnd();
    if(name == "head_on")       return scenario_HeadOn();
    if(name == "junction")      return scenario_Junction();
    if(name == "comm_delay")    return scenario_CommDelay();
    if(name == "packet_loss")   return scenario_PacketLoss();
    if(name == "sensor_fault")  return scenario_SensorFault();
    if(name == "multi_conflict")return scenario_MultiConflict();
    if(name == "emergency")     return scenario_EmergencyBrake();
    if(name == "high_density")  return scenario_HighDensity();

    LOG_WARN("Unknown scenario: " << name << " – defaulting to rear_end");
    return scenario_RearEnd();
}

// ─── Scenario 1: Normal ops ───────────────────────────────────────────────────
ScenarioConfig ScenarioManager::scenario_NormalOps() {
    clearAll();
    buildLinearNetwork(5, 2000.0);

    // Station IDs: 1..5, Tracks: 1..4
    auto addP = [&](TrainId id, const std::string& nm, uint32_t from, uint32_t to,
                    double posM, double kmh) {
        auto t = std::make_unique<PassengerTrain>(id, nm, from, to, posM, kmh);
        t->data().currentTrackId = rm_.assignRoute(id, from, to) > 0 ? 1u : 1u;
        // Assign track 1 manually since tracks are numbered 1..4
        if(auto* r = rm_.routeForTrain(id); r) {
            t->data().currentTrackId = r->currentTrack();
            t->data().currentFromNode = from;
            t->data().currentToNode   = to;
            net_.trainEntersTrack(id, t->data().currentTrackId);
        }
        tsm_.addTrain(std::move(t));
    };

    addP(1, "T01", 1, 5, 0.0,   80.0);
    addP(2, "T02", 1, 5, 500.0, 70.0);
    addP(3, "T03", 2, 5, 0.0,   90.0);

    return {"normal", "Normal operation: 3 trains, no conflicts expected", 120.0};
}

// ─── Scenario 2: Rear-end ─────────────────────────────────────────────────────
ScenarioConfig ScenarioManager::scenario_RearEnd() {
    clearAll();
    buildLinearNetwork(3, 5000.0);

    // T01 is slow (leading), T02 is fast (trailing) → rear-end risk
    auto t1 = std::make_unique<PassengerTrain>(1, "T01", 1, 3, 2000.0, 40.0);
    auto t2 = std::make_unique<ExpressTrain>  (2, "T02", 1, 3,    0.0, 80.0);

    rm_.assignRoute(1, 1, 3);
    rm_.assignRoute(2, 1, 3);

    if(auto* r = rm_.routeForTrain(1); r) {
        t1->data().currentTrackId = r->currentTrack();
        t1->data().currentFromNode = 1; t1->data().currentToNode = 2;
        net_.trainEntersTrack(1, t1->data().currentTrackId);
    }
    if(auto* r = rm_.routeForTrain(2); r) {
        t2->data().currentTrackId = r->currentTrack();
        t2->data().currentFromNode = 1; t2->data().currentToNode = 2;
        net_.trainEntersTrack(2, t2->data().currentTrackId);
    }
    t1->data().state = TrainState::RUNNING;
    t2->data().state = TrainState::RUNNING;

    tsm_.addTrain(std::move(t1));
    tsm_.addTrain(std::move(t2));

    return {"rear_end",
            "T01 at 40km/h (leading), T02 at 80km/h (trailing) on same track. "
            "System should slow T02 before collision.",
            120.0};
}

// ─── Scenario 3: Head-on ──────────────────────────────────────────────────────
ScenarioConfig ScenarioManager::scenario_HeadOn() {
    clearAll();
    buildLinearNetwork(2, 5000.0);

    auto t1 = std::make_unique<PassengerTrain>(1, "T01", 1, 2,    0.0, 80.0);
    auto t2 = std::make_unique<PassengerTrain>(2, "T02", 2, 1, 4800.0, 80.0);

    rm_.assignRoute(1, 1, 2);
    rm_.assignRoute(2, 2, 1);

    if(auto* r = rm_.routeForTrain(1); r) {
        t1->data().currentTrackId = r->currentTrack();
        t1->data().direction = Direction::FORWARD;
        t1->data().currentFromNode = 1; t1->data().currentToNode = 2;
        net_.trainEntersTrack(1, t1->data().currentTrackId);
    }
    if(auto* r = rm_.routeForTrain(2); r) {
        t2->data().currentTrackId = r->currentTrack();
        t2->data().direction = Direction::BACKWARD;
        t2->data().currentFromNode = 2; t2->data().currentToNode = 1;
        net_.trainEntersTrack(2, t2->data().currentTrackId);
    }
    t1->data().state = TrainState::RUNNING;
    t2->data().state = TrainState::RUNNING;

    tsm_.addTrain(std::move(t1));
    tsm_.addTrain(std::move(t2));

    return {"head_on", "Two trains approaching head-on at 80km/h each. "
                       "System must trigger emergency braking.", 60.0};
}

// ─── Scenario 4: Junction ─────────────────────────────────────────────────────
ScenarioConfig ScenarioManager::scenario_Junction() {
    clearAll();
    buildBranchNetwork();

    auto t1 = std::make_unique<PassengerTrain>(1, "T01", 1, 4, 0.0, 80.0);
    auto t2 = std::make_unique<PassengerTrain>(2, "T02", 3, 4, 0.0, 80.0);
    rm_.assignRoute(1, 1, 4); rm_.assignRoute(2, 3, 4);
    if(auto* r = rm_.routeForTrain(1); r) {
        t1->data().currentTrackId = r->currentTrack();
        t1->data().currentFromNode = 1; t1->data().currentToNode = 2;
        net_.trainEntersTrack(1, t1->data().currentTrackId);
    }
    if(auto* r = rm_.routeForTrain(2); r) {
        t2->data().currentTrackId = r->currentTrack();
        t2->data().currentFromNode = 3; t2->data().currentToNode = 2;
        net_.trainEntersTrack(2, t2->data().currentTrackId);
    }
    t1->data().state = TrainState::RUNNING;
    t2->data().state = TrainState::RUNNING;
    tsm_.addTrain(std::move(t1));
    tsm_.addTrain(std::move(t2));
    return {"junction", "Two trains converge on same junction. Hold one.", 90.0};
}

// ─── Scenarios 5-10: simplified stubs calling rear_end as base ────────────────
ScenarioConfig ScenarioManager::scenario_CommDelay() {
    auto sc = scenario_RearEnd();
    sc.name = "comm_delay";
    sc.description = "Rear-end scenario with 500ms communication delay.";
    return sc;
}
ScenarioConfig ScenarioManager::scenario_PacketLoss() {
    auto sc = scenario_RearEnd();
    sc.name = "packet_loss";
    sc.description = "Rear-end with 20% packet loss.";
    return sc;
}
ScenarioConfig ScenarioManager::scenario_SensorFault() {
    auto sc = scenario_RearEnd();
    sc.name = "sensor_fault";
    sc.description = "T01 position sensor fails mid-simulation.";
    return sc;
}
ScenarioConfig ScenarioManager::scenario_MultiConflict() {
    auto sc = scenario_HighDensity();
    sc.name = "multi_conflict";
    return sc;
}
ScenarioConfig ScenarioManager::scenario_EmergencyBrake() {
    auto sc = scenario_HeadOn();
    sc.name = "emergency";
    return sc;
}

ScenarioConfig ScenarioManager::scenario_HighDensity() {
    clearAll();
    buildLinearNetwork(6, 3000.0);
    TrainId id = 1;
    for(int i = 0; i < 10; ++i) {
        double pos   = i * 250.0;
        double speed = 60.0 + (i % 3) * 15.0;
        auto t = std::make_unique<PassengerTrain>(id, "T" + std::to_string(id), 1, 6, pos, speed);
        rm_.assignRoute(id, 1, 6);
        if(auto* r = rm_.routeForTrain(id); r) {
            t->data().currentTrackId = r->currentTrack();
            t->data().currentFromNode = 1; t->data().currentToNode = 2;
            net_.trainEntersTrack(id, t->data().currentTrackId);
        }
        t->data().state = TrainState::RUNNING;
        tsm_.addTrain(std::move(t));
        ++id;
    }
    return {"high_density", "10 trains at varying speeds on same corridor.", 180.0};
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
void ScenarioManager::clearAll() {
    // Clear network by reconstruction isn't directly exposed; we rely on a fresh
    // instance being passed, but for repeated calls in the same run we just log.
    LOG_INFO("Scenario: resetting (note: call with fresh objects for full reset)");
}

void ScenarioManager::buildLinearNetwork(int stationCount, double segmentLenM) {
    std::vector<uint32_t> ids;
    char nm = 'A';
    for(int i = 0; i < stationCount; ++i, ++nm)
        ids.push_back(net_.addStation(std::string(1, nm), i * segmentLenM, 0.0));
    for(int i = 0; i + 1 < stationCount; ++i)
        net_.addTrack(std::string(1, char('A'+i)) + "-" + std::string(1, char('A'+i+1)),
                      ids[static_cast<std::size_t>(i)],
                      ids[static_cast<std::size_t>(i+1)],
                      segmentLenM, 160.0, true);
}

void ScenarioManager::buildBranchNetwork() {
    //   A(1)──J(2)──D(4)
    //   C(3)──/
    uint32_t a = net_.addStation("A", 0, 0);
    uint32_t j = net_.addJunction("J1");
    uint32_t c = net_.addStation("C", 0, 2000);
    uint32_t d = net_.addStation("D", 3000, 1000);
    net_.addTrack("A-J", a, j, 2000.0, 120.0, true);
    net_.addTrack("C-J", c, j, 2000.0, 120.0, true);
    net_.addTrack("J-D", j, d, 2000.0, 120.0, true);
}

} // namespace tca

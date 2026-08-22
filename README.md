# Real-Time Train Collision Avoidance System

A **C++23 / G++14** simulation of a predictive train collision avoidance system.  
Built from first principles: physics → prediction → safety → concurrency → dashboard.

---

## Architecture

```
Railway Network → Train Fleet → Simulation Engine
       ↓                ↓              ↓
  Graph/Routes   Train State Mgr   Fixed Timestep (50ms)
                                       ↓
                               Collision Predictor
                                 (TTC · Sep · Risk)
                                       ↓
                               Conflict Manager
                                       ↓
                               Conflict Resolver
                               (Safety Hierarchy)
                                       ↓
                          Dashboard ←──┤──→ Logger
```

---

## Requirements

| Tool    | Minimum |
|---------|---------|
| G++     | 14+     |
| CMake   | 3.25+   |
| C++     | 23      |
| OS      | Linux   |

---

## Build

```bash
# Clone and enter
cd Realtime-Train_Collision_Avoidance

# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j$(nproc)
```

---

## Run

```bash
# Default: rear-end scenario, 30s
./build/bin/train_sim

# Specific scenario
./build/bin/train_sim --scenario head_on --duration 60

# Real-time multithreaded dashboard
./build/bin/train_sim --scenario rear_end --realtime

# All options
./build/bin/train_sim --help
```

### Available Scenarios

| Scenario        | Description |
|-----------------|-------------|
| `normal`        | 3 trains, normal ops, no conflicts |
| `rear_end`      | T01@40km/h leading, T02@80km/h trailing → auto-slow |
| `head_on`       | Two trains approaching → emergency brake |
| `junction`      | Two trains converging on same junction |
| `comm_delay`    | Rear-end with 500ms comm delay |
| `packet_loss`   | Rear-end with 20% packet loss |
| `sensor_fault`  | T01 position sensor fails |
| `multi_conflict`| 10 trains, multiple simultaneous conflicts |
| `emergency`     | Head-on emergency braking |
| `high_density`  | 10 trains on same corridor |

---

## Tests

```bash
cd build
ctest --output-on-failure
# or individually:
./bin/test_physics
./bin/test_railway
./bin/test_prediction
```

---

## Logs

After each run, `logs/` contains:

| File                  | Contents |
|-----------------------|----------|
| `system.log`          | System events |
| `train_state.csv`     | Per-tick train positions, speeds, states |
| `conflicts.csv`       | All detected conflicts with TTC and risk |
| `control_actions.csv` | All resolver actions applied |

---

## Project Structure

```
├── include/
│   ├── core/          Types, Config, Logger, SimulationClock
│   ├── railway/       Track, Station, Junction, Signal, Network, Routes
│   ├── train/         Train hierarchy, Physics, StateManager
│   ├── simulation/    MovementEngine, SimulationEngine
│   ├── prediction/    CollisionPredictor, StateEstimator
│   ├── safety/        RiskAssessor, ConflictManager, ConflictResolver
│   ├── communication/ CommChannel (delay/jitter/loss), MessageBus
│   └── monitoring/    Dashboard, EventLogger, ScenarioManager
├── src/               Implementation files
├── tests/             Unit + integration tests
├── config/            system.cfg
└── logs/              Runtime output
```

---

## Physics Model

**Euler integration (fixed Δt = 50ms):**

```
x_{t+Δt} = x_t + v_t·Δt + ½·a·Δt²
v_{t+Δt} = v_t + a·Δt
```

**Braking distance:**

```
d_brake = v·t_reaction + v²/(2·a_brake)
```

**Required safe separation:**

```
d_safe = L_train + d_brake + d_margin
```

**TTC (same direction):**

```
TTC = (gap - d_safe) / (v_trailing - v_leading)
```

---

## Safety Hierarchy

```
No action → Speed Advisory → Speed Restriction
         → Controlled Braking → Train Hold → Emergency Braking
```

Uses least-disruptive action that guarantees safe separation.

---

## Team Division

| Member | Module |
|--------|--------|
| 1 | Railway: Network, BFS/DFS/Dijkstra, RouteManager |
| 2 | Train: Hierarchy, Physics, StateManager, Clock |
| 3 | Safety: Predictor, TTC, RiskAssessor, ConflictManager, Resolver |
| 4 | Comms: CommChannel, MessageBus, Delay/Loss/Faults, StateEstimator |
| 5 | Integration: ControlCenter, Dashboard, Logger, Scenarios, CMake |

---

## Disclaimer

This is an **engineering simulation / proof-of-concept**, not a certified ATP/CBTC system.
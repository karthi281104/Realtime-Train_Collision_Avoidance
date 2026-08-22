#pragma once
#include "core/Types.hpp"
#include <string>

namespace tca {

// ─── Train category ───────────────────────────────────────────────────────────
enum class TrainType : uint8_t { PASSENGER, EXPRESS, FREIGHT };
std::string_view toString(TrainType t);

// ─── Physical specification (constant per train) ───────────────────────────────
struct TrainSpec {
    double maxSpeedMs{0.0};           // top speed, m/s
    double normalAccelMs2{0.8};       // cruise accel, m/s²
    double normalBrakeMs2{1.0};       // service brake decel, m/s²
    double emergencyBrakeMs2{2.5};    // emergency decel, m/s²
    double lengthM{200.0};            // train length, metres
    double reactionTimeS{1.5};        // system reaction delay, seconds
    double safetyMarginM{50.0};       // extra buffer beyond braking distance

    static TrainSpec forPassenger();
    static TrainSpec forExpress();
    static TrainSpec forFreight();
};

// ─── Complete mutable runtime state ───────────────────────────────────────────
struct TrainData {
    TrainId   id{0};
    TrainType type{TrainType::PASSENGER};
    TrainSpec spec;

    std::string name;   // "T01", "T02", …

    // ── Kinematics ──────────────────────────────────────────────────────────
    double positionM{0.0};        // metres from current-track origin
    double velocityMs{0.0};       // m/s, always ≥ 0
    double accelerationMs2{0.0};  // m/s² applied this tick (signed)

    // ── Track / route ────────────────────────────────────────────────────────
    TrackId   currentTrackId{0};
    uint32_t  currentFromNode{0};
    uint32_t  currentToNode{0};
    Direction direction{Direction::FORWARD};
    uint32_t  destinationNode{0};

    // ── Control targets ──────────────────────────────────────────────────────
    double targetSpeedMs{0.0};
    bool   emergencyBrakeActive{false};

    // ── State machine ────────────────────────────────────────────────────────
    TrainState state{TrainState::IDLE};

    // ── Sensors ──────────────────────────────────────────────────────────────
    SensorStatus posSensor  {SensorStatus::NORMAL};
    SensorStatus speedSensor{SensorStatus::NORMAL};

    // ── Timestamps ────────────────────────────────────────────────────────────
    double simTimestamp{0.0};   // sim-time of last update
    double lastCommTime{0.0};   // sim-time of last received message

    // ── Helpers ───────────────────────────────────────────────────────────────
    double frontPositionM()  const { return positionM; }
    double rearPositionM()   const { return std::max(0.0, positionM - spec.lengthM); }
    double speedKmh()        const { return velocityMs * kMsToKmh; }
};

// ─── Abstract base class ──────────────────────────────────────────────────────
class Train {
public:
    explicit Train(TrainData d) : data_(std::move(d)) {}
    virtual ~Train() = default;

    // Non-copyable; movable
    Train(const Train&)            = delete;
    Train& operator=(const Train&) = delete;
    Train(Train&&)                 = default;
    Train& operator=(Train&&)      = default;

    TrainData&       data()       { return data_; }
    const TrainData& data() const { return data_; }

    TrainId     id()    const { return data_.id; }
    TrainType   type()  const { return data_.type; }
    TrainState  state() const { return data_.state; }
    std::string name()  const { return data_.name; }

    /// Apply one kinematic tick.  Implemented generically in TrainPhysics.
    virtual void applyPhysics(double dt);

    /// Compute braking distance from current speed.
    double brakingDistance(bool emergency = false) const;

    /// Safe separation required to stop behind a stationary obstacle.
    double requiredSeparation() const;

protected:
    TrainData data_;
};

} // namespace tca

#pragma once
#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

namespace tca {

// ─── Time types ───────────────────────────────────────────────────────────────
using SimTime    = double;          // simulation seconds
using WallClock  = std::chrono::steady_clock;
using TimePoint  = WallClock::time_point;
using Duration   = std::chrono::duration<double>;

// ─── Identifiers ──────────────────────────────────────────────────────────────
using TrainId    = std::uint32_t;
using TrackId    = std::uint32_t;
using StationId  = std::uint32_t;
using JunctionId = std::uint32_t;
using SignalId   = std::uint32_t;
using ConflictId = std::uint64_t;
using RouteId    = std::uint32_t;

// ─── Physical constants ───────────────────────────────────────────────────────
constexpr double kMsToMs        = 1.0 / 3.6;   // km/h → m/s
constexpr double kMsToKmh       = 3.6;          // m/s → km/h
constexpr double kGravity       = 9.81;         // m/s²
constexpr double kInfinity      = 1e18;
constexpr double kEpsilon       = 1e-9;

// ─── Simulation defaults ─────────────────────────────────────────────────────
constexpr double kDefaultDt     = 0.05;         // 50 ms fixed step
constexpr double kPredHorizon   = 30.0;         // 30 second look-ahead
constexpr double kSafetyMargin  = 50.0;         // metres

// ─── Train state ──────────────────────────────────────────────────────────────
enum class TrainState : std::uint8_t {
    IDLE,
    RUNNING,
    WARNING,
    BRAKING,
    STOPPED,
    EMERGENCY_BRAKING
};

std::string_view toString(TrainState s);

// ─── Risk level ───────────────────────────────────────────────────────────────
enum class RiskLevel : std::uint8_t {
    SAFE,
    CAUTION,
    WARNING,
    CRITICAL,
    EMERGENCY
};

std::string_view toString(RiskLevel r);

// ─── Conflict action ──────────────────────────────────────────────────────────
enum class ConflictAction : std::uint8_t {
    NONE,
    SPEED_ADVISORY,
    SPEED_RESTRICTION,
    CONTROLLED_BRAKING,
    TRAIN_HOLD,
    EMERGENCY_BRAKING,
    REROUTE
};

std::string_view toString(ConflictAction a);

// ─── Signal state ─────────────────────────────────────────────────────────────
enum class SignalAspect : std::uint8_t {
    GREEN,
    YELLOW,
    RED,
    FLASHING_YELLOW
};

// ─── Direction ────────────────────────────────────────────────────────────────
enum class Direction : std::uint8_t {
    FORWARD,
    BACKWARD,
    UNKNOWN
};

// ─── Sensor health ────────────────────────────────────────────────────────────
enum class SensorStatus : std::uint8_t {
    NORMAL,
    DEGRADED,
    FAILED
};

} // namespace tca

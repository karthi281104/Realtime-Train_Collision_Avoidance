#pragma once
#include "safety/ConflictManager.hpp"
#include <mutex>
#include <queue>
#include <random>
#include <string>

namespace tca {

/// Simulates a one-hop radio/GSM communication channel with delay, jitter,
/// packet loss, and message age tracking.
struct Message {
    TrainId   senderId{0};
    TrainData payload;
    double    sentAtSimTime{0.0};
    double    arriveAtWallTime{0.0};  // wall-clock deadline
};

class CommChannel {
public:
    /// delayMs: mean one-way delay, jitterMs: ± variation, lossRate: 0..1
    CommChannel(double delayMs = 100.0, double jitterMs = 30.0,
                double lossRate = 0.02, unsigned seed = 42u);

    void send(const TrainData& state, double simNow, double wallNowSeconds = 0.0);

    /// Drain messages whose wall-clock time has arrived.
    std::vector<TrainData> receive(double wallNowSeconds);

    // Fault injection
    void setDelayMs  (double ms)  { delayMs_   = ms; }
    void setJitterMs (double ms)  { jitterMs_  = ms; }
    void setLossRate (double r)   { lossRate_  = r;  }

    double    delayMs()  const { return delayMs_;  }
    double    jitterMs() const { return jitterMs_; }
    double    lossRate() const { return lossRate_; }

private:
    double delayMs_, jitterMs_, lossRate_;
    std::mt19937                   rng_;
    std::normal_distribution<>     jitterDist_;
    std::uniform_real_distribution<> lossDist_;

    std::mutex        mtx_;
    std::vector<Message> pending_;
};

} // namespace tca

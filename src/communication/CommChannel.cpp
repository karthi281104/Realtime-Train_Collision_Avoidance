#include "communication/CommChannel.hpp"
#include <algorithm>

namespace tca {

CommChannel::CommChannel(double delayMs, double jitterMs, double lossRate, unsigned seed)
    : delayMs_(delayMs), jitterMs_(jitterMs), lossRate_(lossRate),
      rng_(seed),
      jitterDist_(0.0, jitterMs),
      lossDist_(0.0, 1.0)
{}

void CommChannel::send(const TrainData& state, double simNow) {
    std::lock_guard lk(mtx_);
    // Packet loss check
    if(lossDist_(rng_) < lossRate_) return;  // dropped

    double jitter = jitterDist_(rng_);
    double totalDelayS = (delayMs_ + jitter) / 1000.0;

    Message msg;
    msg.senderId = state.id;
    msg.payload  = state;
    msg.sentAtSimTime    = simNow;
    msg.arriveAtWallTime = totalDelayS;  // relative delay
    pending_.push_back(std::move(msg));
}

std::vector<TrainData> CommChannel::receive(double wallNowSeconds) {
    std::lock_guard lk(mtx_);
    std::vector<TrainData> arrived;
    auto it = pending_.begin();
    while(it != pending_.end()) {
        if(wallNowSeconds >= it->arriveAtWallTime) {
            arrived.push_back(it->payload);
            it = pending_.erase(it);
        } else {
            ++it;
        }
    }
    return arrived;
}

} // namespace tca

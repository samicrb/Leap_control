#pragma once

#include "sensor/ILeapSource.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace dgd {

// LeapC adapter.
//
// - When compiled with HAVE_LEAPC defined (see CMakeLists.txt), this
//   class creates a LEAP_CONNECTION, spins a polling thread that drains
//   LEAP_TRACKING_EVENT frames, and exposes the latest one via pollLatest().
//
// - When compiled without HAVE_LEAPC, the implementation falls back to a
//   stub that reports sensor_connected = false. This keeps the rest of
//   the pipeline exercisable on a dev laptop without the SDK installed.
//
// All LeapC symbols live in LeapSource.cpp only.
class LeapSource final : public ILeapSource {
public:
    LeapSource();
    ~LeapSource() override;

    bool start() override;
    void stop()  override;
    bool pollLatest(HandFrame& out) override;
    bool isConnected() const override { return connected_.load(); }

private:
    struct Impl;
    std::unique_ptr<Impl> p_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::thread poll_thread_;

    std::mutex           frame_mx_;
    HandFrame            latest_{};
    std::atomic<bool>    has_new_frame_{false};
    double               start_monotonic_s_ = 0.0;

    void pollLoop();
};

} // namespace dgd

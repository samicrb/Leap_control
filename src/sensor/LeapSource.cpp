// LeapSource.cpp
//
// LeapC integration point.
//
// Build modes:
//   HAVE_LEAPC=1  -> real LeapC-based implementation.
//   HAVE_LEAPC=0  -> stub that pretends the sensor is disconnected so the
//                    rest of the pipeline can run on a dev machine.
//
// LeapC API reference: Ultraleap Gemini 6.2.0 documentation. Functions used:
//   LeapCreateConnection, LeapOpenConnection, LeapPollConnection,
//   LeapDestroyConnection, LEAP_TRACKING_EVENT, LEAP_HAND.
//
// If your installed LeapC happens to expose slightly different function
// signatures or event enumerants (rare), update only this file.

#include "sensor/LeapSource.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cstring>

#if defined(HAVE_LEAPC) && HAVE_LEAPC
  // The real LeapC header is only included in this translation unit.
  // Gemini 6.2.0's LeapC.h uses nameless struct/unions (C4201) on MSVC /W4.
  #ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4201)
  #endif
  #include <LeapC.h>
  #ifdef _MSC_VER
    #pragma warning(pop)
  #endif
#endif

namespace dgd {

namespace {
double nowMonotonicSeconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}
} // namespace

struct LeapSource::Impl {
#if defined(HAVE_LEAPC) && HAVE_LEAPC
    LEAP_CONNECTION connection = nullptr;
#endif
};

LeapSource::LeapSource() : p_(std::make_unique<Impl>()) {}
LeapSource::~LeapSource() { stop(); }

bool LeapSource::start() {
    start_monotonic_s_ = nowMonotonicSeconds();

#if defined(HAVE_LEAPC) && HAVE_LEAPC
    eLeapRS rs = LeapCreateConnection(nullptr, &p_->connection);
    if (rs != eLeapRS_Success) {
        LOG_E("LeapCreateConnection failed (rs=%d)", static_cast<int>(rs));
        return false;
    }
    rs = LeapOpenConnection(p_->connection);
    if (rs != eLeapRS_Success) {
        LOG_E("LeapOpenConnection failed (rs=%d)", static_cast<int>(rs));
        LeapDestroyConnection(p_->connection);
        p_->connection = nullptr;
        return false;
    }
    running_.store(true);
    poll_thread_ = std::thread(&LeapSource::pollLoop, this);
    LOG_I("LeapC: connection opened, polling thread started.");
    return true;
#else
    LOG_W("LeapSource built without HAVE_LEAPC - sensor is STUB only.");
    connected_.store(false);
    running_.store(false);
    return false;
#endif
}

void LeapSource::stop() {
    running_.store(false);
    if (poll_thread_.joinable()) poll_thread_.join();

#if defined(HAVE_LEAPC) && HAVE_LEAPC
    if (p_ && p_->connection) {
        LeapCloseConnection(p_->connection);
        LeapDestroyConnection(p_->connection);
        p_->connection = nullptr;
    }
#endif
    connected_.store(false);
}

bool LeapSource::pollLatest(HandFrame& out) {
    std::lock_guard<std::mutex> lock(frame_mx_);
    out = latest_;
    bool was_new = has_new_frame_.exchange(false);
    return was_new;
}

#if defined(HAVE_LEAPC) && HAVE_LEAPC

namespace {

std::array<double, 3> vecFromLeap(const LEAP_VECTOR& v) {
    return { static_cast<double>(v.x),
             static_cast<double>(v.y),
             static_cast<double>(v.z) };
}

void fillHandSample(const LEAP_HAND& h, double ts, HandSample& s) {
    s.palm_position  = vecFromLeap(h.palm.position);
    s.palm_normal    = vecFromLeap(h.palm.normal);
    s.palm_direction = vecFromLeap(h.palm.direction);
    s.grab_strength  = static_cast<double>(h.grab_strength);
    // Some LeapC builds expose "confidence" directly; otherwise use
    // pinch/grab confidence proxy. We cap to [0,1].
    s.confidence     = static_cast<double>(h.confidence);
    if (s.confidence < 0.0) s.confidence = 0.0;
    if (s.confidence > 1.0) s.confidence = 1.0;
    s.timestamp_s    = ts;
}

} // namespace

void LeapSource::pollLoop() {
    LEAP_CONNECTION_MESSAGE msg;
    while (running_.load()) {
        // 50 ms timeout keeps the loop responsive to stop() without
        // hammering the CPU.
        eLeapRS rs = LeapPollConnection(p_->connection, 50, &msg);
        if (rs != eLeapRS_Success) {
            if (rs == eLeapRS_Timeout) continue;
            LOG_W("LeapPollConnection error (rs=%d)", static_cast<int>(rs));
            continue;
        }

        if (msg.type == eLeapEventType_Connection) {
            connected_.store(true);
            LOG_I("LeapC: device connection event received.");
        } else if (msg.type == eLeapEventType_ConnectionLost) {
            connected_.store(false);
            LOG_W("LeapC: connection lost.");
        } else if (msg.type == eLeapEventType_Tracking) {
            const LEAP_TRACKING_EVENT* te = msg.tracking_event;
            if (!te) continue;

            HandFrame frame;
            frame.sensor_connected = true;
            frame.timestamp_s      = nowMonotonicSeconds() - start_monotonic_s_;
            frame.device_id        = te->info.frame_id;

            for (uint32_t i = 0; i < te->nHands; ++i) {
                const LEAP_HAND& h = te->pHands[i];
                HandSample s;
                fillHandSample(h, frame.timestamp_s, s);
                if (h.type == eLeapHandType_Left)       frame.left  = s;
                else if (h.type == eLeapHandType_Right) frame.right = s;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mx_);
                latest_ = frame;
            }
            has_new_frame_.store(true);
        }
    }
}

#else // !HAVE_LEAPC

void LeapSource::pollLoop() { /* stub build: no polling */ }

#endif

} // namespace dgd

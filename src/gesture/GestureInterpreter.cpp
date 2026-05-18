#include "gesture/GestureInterpreter.hpp"
#include "util/Logger.hpp"

#include <cmath>

namespace dgd {

GestureInterpreter::GestureInterpreter(const Config& cfg) : cfg_(cfg) {}

void GestureInterpreter::reset() {
    reference_set_ = false;
    smoother_primed_ = false;
    last_left_s_ = -1.0;
    last_right_s_ = -1.0;
    last_left_posture_ = HandPosture::Unknown;
    last_right_posture_ = HandPosture::Unknown;
    gripper_last_zone_ = GripperZone::Unknown;
    gripper_armed_since_s_ = -1.0;
}

void GestureInterpreter::captureReference() {
    if (!smoother_primed_) {
        // Refuse to set a reference on a nonexistent hand - state
        // machine should not call us in that case, but stay safe.
        reference_set_ = false;
        return;
    }
    ref_right_position_  = smoothed_position_;
    ref_right_normal_    = smoothed_normal_;
    ref_right_direction_ = smoothed_direction_;
    reference_set_ = true;
    LOG_D("Gesture: reference captured at (%.1f, %.1f, %.1f)",
          ref_right_position_[0], ref_right_position_[1], ref_right_position_[2]);
}

HandPosture GestureInterpreter::posture(double grab, HandPosture prev) const {
    // Hysteresis: require full crossing of the opposite threshold to flip.
    if (prev == HandPosture::Closed) {
        if (grab < cfg_.grab_open_threshold)   return HandPosture::Open;
        return HandPosture::Closed;
    }
    if (prev == HandPosture::Open) {
        if (grab > cfg_.grab_closed_threshold) return HandPosture::Closed;
        return HandPosture::Open;
    }
    // From Unknown: commit to the nearer side confidently, else leave it.
    if (grab > cfg_.grab_closed_threshold) return HandPosture::Closed;
    if (grab < cfg_.grab_open_threshold)   return HandPosture::Open;
    return HandPosture::Unknown;
}

namespace {

// Approximate XYZ Euler delta from two (direction, normal) basis samples.
// This is an intentional simplification: full quaternion math is overkill
// for a demo where orientation is already heavily bounded and scaled.
Vec3 orientationDelta(const Vec3& dir_ref, const Vec3& nrm_ref,
                      const Vec3& dir_cur, const Vec3& nrm_cur) {
    // Yaw = angle between projections of direction on the XZ plane.
    auto angleXZ = [](const Vec3& v) {
        return std::atan2(v[0], -v[2]) * kRad2Deg;
    };
    // Pitch = angle of direction vs XZ plane (Y component).
    auto pitch = [](const Vec3& v) {
        double len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (len < 1e-6) return 0.0;
        return std::asin(clamp(v[1] / len, -1.0, 1.0)) * kRad2Deg;
    };
    // Roll = tilt of palm normal around the hand direction.
    auto roll = [](const Vec3& n) {
        return std::atan2(n[0], -n[1]) * kRad2Deg;
    };

    double dx = pitch(dir_cur)   - pitch(dir_ref);   // rx
    double dy = angleXZ(dir_cur) - angleXZ(dir_ref); // ry
    double dz = roll(nrm_cur)    - roll(nrm_ref);    // rz

    // Wrap each component to [-180, 180].
    auto wrap = [](double a) {
        while (a >  180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    };
    return { wrap(dx), wrap(dy), wrap(dz) };
}

} // namespace

GestureReport GestureInterpreter::update(const HandFrame& frame, double now_s) {
    GestureReport r;
    r.timestamp_s = now_s;
    r.sensor_ok = frame.sensor_connected;
    r.freshFrame = frame.sensor_connected;

    // --- Presence / confidence -----------------------------------------
    if (frame.left && frame.left->confidence >= cfg_.min_confidence) {
        r.leftPresent = true;
        r.leftConfidence = frame.left->confidence;
        last_left_s_ = now_s;
        last_left_posture_ = posture(frame.left->grab_strength, last_left_posture_);
    } else if (now_s - last_left_s_ < cfg_.hand_loss_timeout_s && last_left_s_ > 0.0) {
        // Brief blip - keep the previous presence verdict stable.
        r.leftPresent = true;
        r.leftConfidence = 0.0;
    } else {
        r.leftPresent = false;
        last_left_posture_ = HandPosture::Unknown;
    }
    r.leftPosture = last_left_posture_;

    if (frame.right && frame.right->confidence >= cfg_.min_confidence) {
        r.rightPresent = true;
        r.rightConfidence = frame.right->confidence;
        last_right_s_ = now_s;
        last_right_posture_ = posture(frame.right->grab_strength, last_right_posture_);

        // Update smoothed right-hand sample.
        const auto& s = *frame.right;
        if (!smoother_primed_) {
            smoothed_position_  = s.palm_position;
            smoothed_normal_    = s.palm_normal;
            smoothed_direction_ = s.palm_direction;
            smoother_primed_ = true;
        } else {
            smoothed_position_  = emaVec(smoothed_position_,  s.palm_position,  cfg_.smoothing_alpha);
            smoothed_normal_    = emaVec(smoothed_normal_,    s.palm_normal,    cfg_.smoothing_alpha);
            smoothed_direction_ = emaVec(smoothed_direction_, s.palm_direction, cfg_.smoothing_alpha);
        }
    } else if (now_s - last_right_s_ < cfg_.hand_loss_timeout_s && last_right_s_ > 0.0) {
        r.rightPresent = true;
        r.rightConfidence = 0.0;
    } else {
        r.rightPresent = false;
        last_right_posture_ = HandPosture::Unknown;
        smoother_primed_ = false;
        reference_set_ = false;
    }
    r.rightPosture = last_right_posture_;

    // --- Relative right-hand deltas ------------------------------------
    if (reference_set_ && smoother_primed_) {
        Vec3 d = smoothed_position_ - ref_right_position_;

        // Apply dead-zone axis by axis (mm).
        d[0] = deadzone(d[0], cfg_.position_deadzone_mm);
        d[1] = deadzone(d[1], cfg_.position_deadzone_mm);
        d[2] = deadzone(d[2], cfg_.position_deadzone_mm);
        r.rightDeltaPosition = d;

        Vec3 rot = orientationDelta(ref_right_direction_, ref_right_normal_,
                                    smoothed_direction_,  smoothed_normal_);
        rot[0] = deadzone(rot[0], cfg_.orientation_deadzone_deg);
        rot[1] = deadzone(rot[1], cfg_.orientation_deadzone_deg);
        rot[2] = deadzone(rot[2], cfg_.orientation_deadzone_deg);
        r.rightDeltaOrientation = rot;
    }

    // --- Gripper gesture ------------------------------------------------
    if (frame.left && frame.right) {
        const auto& L = *frame.left;
        const auto& R = *frame.right;
        double dot_n = dot(L.palm_normal, R.palm_normal);
        Vec3 diff = L.palm_position - R.palm_position;
        double dist = norm(diff);
        r.handDistance_mm = dist;

        bool facing = dot_n < cfg_.gripper_facing_dot_max;
        if (facing) {
            if (gripper_armed_since_s_ < 0.0) gripper_armed_since_s_ = now_s;
            double hold = now_s - gripper_armed_since_s_;
            r.gripperGestureArmed = hold >= cfg_.gripper_gesture_hold_s;
        } else {
            gripper_armed_since_s_ = -1.0;
        }

        if (r.gripperGestureArmed) {
            GripperZone zone;
            if      (dist > cfg_.gripper_open_mm)         zone = GripperZone::Open;
            else if (dist < cfg_.gripper_close_mm)        zone = GripperZone::Close;
            else if (dist > cfg_.gripper_neutral_min_mm &&
                     dist < cfg_.gripper_neutral_max_mm) zone = GripperZone::Neutral;
            else                                          zone = GripperZone::Unknown;

            bool cooldown_ok =
                (now_s - gripper_last_command_s_) > cfg_.gripper_cooldown_s;

            // Only fire when we crossed from Neutral (or Unknown after a
            // neutral pass) into Open/Close. This enforces the anti-repeat
            // rule from section 15 of the context.
            if (cooldown_ok && zone == GripperZone::Open &&
                (gripper_last_zone_ == GripperZone::Neutral ||
                 gripper_last_zone_ == GripperZone::Close)) {
                r.gripperOpenImpulse = true;
                gripper_last_command_s_ = now_s;
                LOG_I("Gripper IMPULSE: OPEN dist=%.1f mm threshold_open=%.1f mm -> O%02d",
                      dist, cfg_.gripper_open_mm, cfg_.gripper_open_do_index);
            } else if (cooldown_ok && zone == GripperZone::Close &&
                       (gripper_last_zone_ == GripperZone::Neutral ||
                        gripper_last_zone_ == GripperZone::Open)) {
                r.gripperCloseImpulse = true;
                gripper_last_command_s_ = now_s;
                LOG_I("Gripper IMPULSE: CLOSE dist=%.1f mm threshold_close=%.1f mm -> O%02d",
                      dist, cfg_.gripper_close_mm, cfg_.gripper_close_do_index);
            }
            gripper_last_zone_ = zone;
        } else {
            gripper_last_zone_ = GripperZone::Unknown;
        }
    } else {
        r.gripperGestureArmed = false;
        r.handDistance_mm = -1.0;
        gripper_armed_since_s_ = -1.0;
        gripper_last_zone_ = GripperZone::Unknown;
    }

    return r;
}

} // namespace dgd

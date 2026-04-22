#pragma once

// Tiny math helpers. Kept header-only and dependency-free so the gesture
// interpreter stays simple. If the project grows, replace with Eigen.

#include <array>
#include <cmath>

namespace dgd {

using Vec3 = std::array<double, 3>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline Vec3 operator*(const Vec3& a, double s)      { return {a[0]*s, a[1]*s, a[2]*s}; }

inline double dot(const Vec3& a, const Vec3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline double norm(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline double sign(double v) { return (v > 0) - (v < 0); }

// Apply a symmetric dead-zone of width 2*dz centred on 0.
inline double deadzone(double v, double dz) {
    if (std::fabs(v) <= dz) return 0.0;
    return v - sign(v) * dz;
}

// Exponential moving average: out = (1 - alpha) * new + alpha * prev.
// alpha in [0,1). 0 = no smoothing, 0.9 = heavy. Keep small for responsive feel.
inline double ema(double prev, double sample, double alpha) {
    return alpha * prev + (1.0 - alpha) * sample;
}

inline Vec3 emaVec(const Vec3& prev, const Vec3& sample, double alpha) {
    return { ema(prev[0], sample[0], alpha),
             ema(prev[1], sample[1], alpha),
             ema(prev[2], sample[2], alpha) };
}

} // namespace dgd

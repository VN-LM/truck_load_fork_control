#pragma once

#include <cmath>
#include <cstdint>

namespace tlf {

struct Vec2 {
  double x{0.0};
  double z{0.0};
};

inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.z + b.z}; }
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.z - b.z}; }
inline Vec2 operator*(double s, const Vec2& v) { return {s * v.x, s * v.z}; }
inline Vec2 operator*(const Vec2& v, double s) { return {s * v.x, s * v.z}; }

struct Rot2 {
  double c{1.0};
  double s{0.0};

  static Rot2 fromRad(double rad) {
    return {std::cos(rad), std::sin(rad)};
  }

  Vec2 apply(const Vec2& v) const {
    // rotation in x-z plane (about y axis)
    return {c * v.x - s * v.z, s * v.x + c * v.z};
  }
};

}  // namespace tlf

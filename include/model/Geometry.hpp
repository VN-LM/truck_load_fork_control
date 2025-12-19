#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "model/Math2D.hpp"

namespace tlf {

enum class CornerId : int {
  RearBottom = 0,
  RearTop = 1,
  FrontBottom = 2,
  FrontTop = 3,
};

struct Plane {
  // ax + by + cz + d = 0
  double a{0.0};
  double b{0.0};
  double c{1.0};
  double d{0.0};

  bool valid() const { return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d) && std::abs(c) > 1e-9; }

  double zAtX(double x) const {
    // assume y=0 for MVP
    return -(a * x + d) / c;
  }
};

struct RackParams {
  double height_m{2.3};
  double length_m{2.3};

  // Vector from fork pivot to rack rear-bottom corner, expressed in rack frame at zero angles.
  // MVP assumes it rotates with rack (i.e., same frame as rack rectangle).
  Vec2 mount_offset_m{0.2, 0.0};
};

struct ForkliftParams {
  // Mast base (tilt pivot) height above the local floor at x=s_m.
  // Used to compute the world pose of the carriage pivot from (s_m, lift_m, pitch, tilt) plus floor profile.
  double mast_pivot_height_m{0.0};
};

struct CornerPoints2D {
  std::array<Vec2, 4> p;
};

struct ClearanceResult {
  double clearance_top_m{0.0};
  double clearance_bottom_m{0.0};
  CornerId worst_point{CornerId::RearBottom};
};

struct EnvironmentGeometry {
  // Either provide scalar heights OR planes. If both are provided, planes take precedence.
  std::optional<double> ceiling_z_m;
  std::optional<double> floor_z_m;

  std::optional<Plane> ceiling_plane;
  std::optional<Plane> floor_plane;

  // Optional callbacks (override scalars). Useful for ramps and piecewise container geometry.
  // If provided, they take precedence over ceiling_z_m/floor_z_m.
  std::function<double(double)> ceiling_z_at_x_m;
  std::function<double(double)> floor_z_at_x_m;
};

// Kinematics contract (2D side view):
// - s_m: mast base x in world.
// - pitch_rad: chassis pitch.
// - tilt_rad: mast tilt relative to chassis.
// - lift_m: carriage travel along mast (+z in rack frame), NOT world-z.
// The carriage pivot world position is:
//   mast_base = (s_m, floor_z_at_x(s_m) + forklift.mast_pivot_height_m)
//   pivot     = mast_base + R(pitch+tilt) * (0, lift_m)
CornerPoints2D computeRackCorners2D(double s_m,
                                   double lift_m,
                                   double pitch_rad,
                                   double tilt_rad,
                                   const EnvironmentGeometry& env,
                                   const RackParams& rack,
                                   const ForkliftParams& forklift);

ClearanceResult computeClearances(const CornerPoints2D& corners,
                                 const EnvironmentGeometry& env,
                                 double margin_top_m,
                                 double margin_bottom_m);

std::string toString(CornerId id);

}  // namespace tlf

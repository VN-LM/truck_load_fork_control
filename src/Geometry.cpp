#include "model/Geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlf {

static double envCeilingZAtX(const EnvironmentGeometry& env, double x) {
  if (env.ceiling_z_at_x_m) {
    return env.ceiling_z_at_x_m(x);
  }
  if (env.ceiling_plane && env.ceiling_plane->valid()) {
    return env.ceiling_plane->zAtX(x);
  }
  return env.ceiling_z_m.value_or(10.0);  // very high by default
}

static double envFloorZAtX(const EnvironmentGeometry& env, double x) {
  if (env.floor_z_at_x_m) {
    return env.floor_z_at_x_m(x);
  }
  if (env.floor_plane && env.floor_plane->valid()) {
    return env.floor_plane->zAtX(x);
  }
  return env.floor_z_m.value_or(0.0);
}

CornerPoints2D computeRackCorners2D(double s_m,
                                   double lift_m,
                                   double pitch_rad,
                                   double tilt_rad,
                                   const EnvironmentGeometry& env,
                                   const RackParams& rack,
                                   const ForkliftParams& forklift) {
  const double theta = pitch_rad + tilt_rad;
  const Rot2 R = Rot2::fromRad(theta);

  // Mast base at local floor + fixed pivot height.
  const double base_z = envFloorZAtX(env, s_m) + forklift.mast_pivot_height_m;
  const Vec2 mast_base{s_m, base_z};

  // Carriage (fork pivot) moves along mast (+z in rack frame).
  const Vec2 pivot_world = mast_base + R.apply(Vec2{0.0, lift_m});

  // Rear-bottom corner position
  const Vec2 rb = pivot_world + R.apply(rack.mount_offset_m);

  const Vec2 rt = rb + R.apply(Vec2{0.0, rack.height_m});
  const Vec2 fb = rb + R.apply(Vec2{rack.length_m, 0.0});
  const Vec2 ft = rb + R.apply(Vec2{rack.length_m, rack.height_m});

  return CornerPoints2D{{rb, rt, fb, ft}};
}

ClearanceResult computeClearances(const CornerPoints2D& corners,
                                 const EnvironmentGeometry& env,
                                 double margin_top_m,
                                 double margin_bottom_m) {
  struct Candidate {
    CornerId id;
    double value;
  };

  // Top clearance: min over top corners of (ceiling - z)
  Candidate topWorst{CornerId::RearTop, std::numeric_limits<double>::infinity()};
  for (CornerId id : {CornerId::RearTop, CornerId::FrontTop}) {
    const auto& p = corners.p[static_cast<int>(id)];
    const double c = envCeilingZAtX(env, p.x) - p.z;
    if (c < topWorst.value) topWorst = {id, c};
  }

  // Bottom clearance: min over bottom corners of (z - floor)
  Candidate botWorst{CornerId::RearBottom, std::numeric_limits<double>::infinity()};
  for (CornerId id : {CornerId::RearBottom, CornerId::FrontBottom}) {
    const auto& p = corners.p[static_cast<int>(id)];
    const double c = p.z - envFloorZAtX(env, p.x);
    if (c < botWorst.value) botWorst = {id, c};
  }

  const double clearance_top_m = topWorst.value - margin_top_m;
  const double clearance_bottom_m = botWorst.value - margin_bottom_m;

  const CornerId worst = (clearance_top_m < clearance_bottom_m) ? topWorst.id : botWorst.id;

  ClearanceResult out;
  out.clearance_top_m = clearance_top_m;
  out.clearance_bottom_m = clearance_bottom_m;
  out.top_worst_point = topWorst.id;
  out.bottom_worst_point = botWorst.id;
  out.worst_point = worst;
  return out;
}

std::string toString(CornerId id) {
  switch (id) {
    case CornerId::RearBottom:
      return "RearBottom";
    case CornerId::RearTop:
      return "RearTop";
    case CornerId::FrontBottom:
      return "FrontBottom";
    case CornerId::FrontTop:
      return "FrontTop";
    default:
      return "Unknown";
  }
}

}  // namespace tlf

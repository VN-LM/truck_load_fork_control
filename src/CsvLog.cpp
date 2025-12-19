#include "utils/CsvLog.hpp"

#include <iomanip>
#include <cmath>

namespace tlf {

static int toInt(SafetyLevel l) { return static_cast<int>(l); }
static int toInt(TerrainState t) { return static_cast<int>(t); }
static int toInt(CornerId c) { return static_cast<int>(c); }

CsvLogger::CsvLogger(std::string path) : out_(std::move(path)) {}

void CsvLogger::writeHeader() {
  out_ << "time,s,pitch,pitch_rate,lift,tilt,ceiling_z,floor_z,"
          "rb_x,rb_z,rt_x,rt_z,fb_x,fb_z,ft_x,ft_z,"
          "clearance_top,clearance_bottom,"
          "lift_cmd,tilt_cmd,speed_limit,"
          "safety_level,terrain_state,worst_point_id\n";
}

void CsvLogger::writeFrame(const DebugFrame& f) {
  out_ << std::fixed << std::setprecision(6);

  const auto& in = f.in;
  const auto& c = f.corners.p;

  auto ceilingAt = [&](double x) {
    if (in.env.ceiling_z_at_x_m) return in.env.ceiling_z_at_x_m(x);
    if (in.env.ceiling_plane && in.env.ceiling_plane->valid()) return in.env.ceiling_plane->zAtX(x);
    return in.env.ceiling_z_m.value_or(10.0);
  };
  auto floorAt = [&](double x) {
    if (in.env.floor_z_at_x_m) return in.env.floor_z_at_x_m(x);
    if (in.env.floor_plane && in.env.floor_plane->valid()) return in.env.floor_plane->zAtX(x);
    return in.env.floor_z_m.value_or(0.0);
  };

  const double ceiling = ceilingAt(in.s_m);
  const double floor = floorAt(in.s_m);

  out_ << f.time_s << ',' << in.s_m << ',' << in.pitch_rad << ',' << in.pitch_rate_rad_s << ','
       << in.lift_pos_m << ',' << in.tilt_rad << ',' << ceiling << ',' << floor << ','
       << c[0].x << ',' << c[0].z << ','
       << c[1].x << ',' << c[1].z << ','
       << c[2].x << ',' << c[2].z << ','
       << c[3].x << ',' << c[3].z << ','
       << f.safety.clearance_top_m << ',' << f.safety.clearance_bottom_m << ','
       << f.cmd.lift_target_m << ',' << f.cmd.tilt_target_rad << ',' << f.cmd.speed_limit_m_s << ','
       << toInt(f.safety.level) << ',' << toInt(in.terrain) << ',' << toInt(f.safety.worst_point)
       << '\n';
}

}  // namespace tlf

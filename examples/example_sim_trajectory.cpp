#include <cmath>
#include <iostream>
#include <string>

#include "controller/Controller.hpp"
#include "utils/CsvLog.hpp"

using namespace tlf;

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

struct SimState {
  double time_s{0.0};
  double s_m{-2.6};
  double pitch_rad{0.0};
  double pitch_rate_rad_s{0.0};
  double lift_m{0.15};  // Initial lift amount (relative to pivot base), not world z.
  double tilt_rad{0.0};
  TerrainState terrain{TerrainState::Ground};
};

struct EnvSpec {
  double door_x_m{0.0};
  double container_len_m{8.0};
  double container_h_m{2.5};
  double ramp_len_m{2.5};
  double ramp_slope_deg{4.0};
  double ground_len_m{4.0};
};

static double floorZAtX(const EnvSpec& e, double x_m) {
  const double h = std::tan(e.ramp_slope_deg * M_PI / 180.0) * e.ramp_len_m;
  const double groundZ = -h;
  const double rampStartX = e.door_x_m - e.ramp_len_m;

  // inside container (extends +x from door)
  if (x_m >= e.door_x_m && x_m <= (e.door_x_m + e.container_len_m)) return 0.0;

  // on ground beyond ramp start (further outside)
  if (x_m <= rampStartX) return groundZ;

  // on ramp: interpolate between rampStartX (groundZ) and door (0)
  if (x_m > rampStartX && x_m < e.door_x_m) {
    const double t = (x_m - rampStartX) / (e.door_x_m - rampStartX);
    return (1.0 - t) * groundZ + t * 0.0;
  }

  // outside (beyond container far end): assume container floor
  return 0.0;
}

static double ceilingZAtX(const EnvSpec& e, double x_m) {
  if (x_m >= e.door_x_m && x_m <= (e.door_x_m + e.container_len_m)) return e.container_h_m;
  return 100.0;
}

static double pitchFromWheelContact(const EnvSpec& e,
                                   double mast_x_m,
                                   double wheelbase_m,
                                   double rear_to_mast_m) {
  // Vehicle heads +x into container: wheels are behind mast in -x.
  // "Near" wheel is closer to mast (more forward), "far" wheel is further outside.
  const double x_near = mast_x_m - rear_to_mast_m;
  const double x_far = x_near - wheelbase_m;
  const double z_near = floorZAtX(e, x_near);
  const double z_far = floorZAtX(e, x_far);
  return std::atan2(z_near - z_far, x_near - x_far);
}

static TerrainState terrainFromPitch(double pitch_rad) {
  const double deg = std::abs(pitch_rad) * 180.0 / M_PI;
  if (deg < 0.5) return TerrainState::Ground;
  return TerrainState::OnRamp;
}

int main(int argc, char** argv) {
  std::string out_path = "/tmp/tlf_log.csv";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--out" && i + 1 < argc) out_path = argv[++i];
  }

  Controller controller;
  {
    ControllerConfig cfg;
    cfg.margin_top_m = 0.08;
    cfg.margin_bottom_m = 0.08;
    cfg.warn_threshold_m = 0.20;

    cfg.search_lift_half_range_m = 0.20;
    cfg.search_tilt_half_range_rad = 0.25;
    cfg.grid_lift_steps = 41;
    cfg.grid_tilt_steps = 41;

    cfg.lookahead_s_m = 0.25;

    cfg.base_lift_rate_limit_m_s = 0.35;
    cfg.base_tilt_rate_limit_rad_s = 0.55;
    controller = Controller(cfg);
  }
  controller.reset();

  CsvLogger log(out_path);
  if (!log.good()) {
    std::cerr << "Failed to open log: " << out_path << "\n";
    return 1;
  }
  log.writeHeader();

  SimState st;

  const EnvSpec envSpec;

  RackParams rack;
  rack.height_m = 2.3;
  rack.length_m = 2.3;
  rack.mount_offset_m = {0.25, 0.05};

  ForkliftParams fl;

  const double dt = 0.1;
  const double v = 0.35;  // base forward speed

  // Vehicle geometry used for pitch-from-wheel-contact model.
  const double wheelbase_m = 2.0;
  const double rear_to_mast_m = 0.1;
  const double pivot_height_above_floor_m = 0.2;  // Mast pivot offset above local floor.

  fl.mast_pivot_height_m = pivot_height_above_floor_m;

  for (int k = 0; k < 6000; ++k) {
    const double pitch = pitchFromWheelContact(envSpec, st.s_m, wheelbase_m, rear_to_mast_m);
    const double pitch_rate = (pitch - st.pitch_rad) / dt;

    const TerrainState terr = terrainFromPitch(pitch);

    EnvironmentGeometry env;
    env.ceiling_z_at_x_m = [&](double x) { return ceilingZAtX(envSpec, x); };
    env.floor_z_at_x_m = [&](double x) { return floorZAtX(envSpec, x); };
    env.ceiling_z_m = ceilingZAtX(envSpec, st.s_m);
    env.floor_z_m = floorZAtX(envSpec, st.s_m);

    ControlInput in;
    in.dt_s = dt;
    in.s_m = st.s_m;
    in.pitch_rad = pitch;
    in.pitch_rate_rad_s = pitch_rate;
    in.terrain = terr;

    // Kinematics contract: lift_pos_m is carriage travel along mast (meters).
    in.lift_pos_m = st.lift_m;
    in.tilt_rad = st.tilt_rad;

    in.env = env;
    in.rack = rack;
    in.forklift = fl;
    in.inputs_valid = true;

    const DebugFrame fr = controller.step(in);

    // simple actuator following: rate-limited towards targets
    const double lift_err = fr.cmd.lift_target_m - st.lift_m;
    const double lift_step = clamp(lift_err, -fr.cmd.lift_rate_limit_m_s * dt, fr.cmd.lift_rate_limit_m_s * dt);
    st.lift_m += lift_step;

    const double tilt_err = fr.cmd.tilt_target_rad - st.tilt_rad;
    const double tilt_step = clamp(tilt_err, -fr.cmd.tilt_rate_limit_rad_s * dt, fr.cmd.tilt_rate_limit_rad_s * dt);
    st.tilt_rad += tilt_step;

    // move forward with speed limit
    const double speed = std::min(v, fr.cmd.speed_limit_m_s);
    // Move into container (+x): s increases.
    st.s_m += speed * dt;

    st.time_s += dt;
    st.pitch_rate_rad_s = pitch_rate;
    st.pitch_rad = pitch;
    st.terrain = terr;

    log.writeFrame(fr);

    if (st.s_m > 3) break;
  }

  std::cout << "Wrote log: " << out_path << "\n";
  return 0;
}

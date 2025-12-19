#include <catch2/catch_test_macros.hpp>

#include "controller/Controller.hpp"

using namespace tlf;

TEST_CASE("Controller finds feasible target when space allows") {
  ControllerConfig cfg;
  cfg.margin_top_m = 0.05;
  cfg.margin_bottom_m = 0.05;
  cfg.search_lift_half_range_m = 0.2;
  cfg.search_tilt_half_range_rad = 0.15;

  Controller c(cfg);

  ControlInput in;
  in.dt_s = 0.02;
  in.s_m = 0.0;
  in.pitch_rad = 0.0;
  in.pitch_rate_rad_s = 0.0;
  in.terrain = TerrainState::InContainer;

  // With rack height 2.3m and ceiling 2.5m, pivot lift must be low enough to be feasible.
  in.lift_pos_m = 0.10;
  in.tilt_rad = 0.0;

  in.env.floor_z_m = 0.0;
  in.env.ceiling_z_m = 2.5;

  in.rack.height_m = 2.3;
  in.rack.length_m = 2.3;
  in.rack.mount_offset_m = {0.0, 0.0};

  const auto f = c.step(in);
  REQUIRE(f.had_feasible_solution);
  REQUIRE(f.safety.level != SafetyLevel::STOP);
}

TEST_CASE("Controller enters DEGRADED on invalid inputs") {
  Controller c;
  ControlInput in;
  in.inputs_valid = false;
  in.env.ceiling_z_m = 2.5;
  in.env.floor_z_m = 0.0;

  const auto f = c.step(in);
  REQUIRE(f.safety.level == SafetyLevel::DEGRADED);
}

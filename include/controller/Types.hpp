#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "model/Geometry.hpp"

namespace tlf {

enum class TerrainState : int {
  Ground = 0,
  FrontOnRamp = 1,
  OnRamp = 2,
  FrontInContainerRearOnRamp = 3,
  InContainer = 4,
};

enum class SafetyLevel : int {
  OK = 0,
  WARN = 1,
  STOP = 2,
  DEGRADED = 3,
};

enum class SafetyCode : int {
  None = 0,
  ClearanceHardViolated = 1,
  ClearanceSoftNear = 2,
  InputInvalid = 3,
  PitchJitter = 4,
  NoFeasibleSolution = 5,
};

struct ControlInput {
  double dt_s{0.02};

  double pitch_rad{0.0};
  double pitch_rate_rad_s{0.0};

  double s_m{0.0};
  TerrainState terrain{TerrainState::Ground};

  // Current actuator states
  double lift_pos_m{0.0};
  double tilt_rad{0.0};

  EnvironmentGeometry env;

  RackParams rack;
  ForkliftParams forklift;

  // Optional: if false, controller should enter DEGRADED.
  bool inputs_valid{true};
};

struct ControlCommand {
  double lift_target_m{0.0};
  double lift_rate_limit_m_s{0.2};

  double tilt_target_rad{0.0};
  double tilt_rate_limit_rad_s{0.4};

  double speed_limit_m_s{1.0};
};

struct SafetyStatus {
  SafetyLevel level{SafetyLevel::OK};
  SafetyCode code{SafetyCode::None};
  std::string message;

  double clearance_top_m{0.0};
  double clearance_bottom_m{0.0};
  CornerId worst_point{CornerId::RearBottom};
};

struct DebugFrame {
  double time_s{0.0};

  ControlInput in;
  ControlCommand cmd;
  SafetyStatus safety;

  CornerPoints2D corners;

  // Candidate selection
  double selected_cost{0.0};
  bool had_feasible_solution{false};
};

struct ControllerConfig {
  // Margins and thresholds
  double margin_top_m{0.08};
  double margin_bottom_m{0.08};
  double warn_threshold_m{0.12};
  double hard_threshold_m{0.00};

  // Search neighborhood
  double search_lift_half_range_m{0.12};
  double search_tilt_half_range_rad{0.10};
  int grid_lift_steps{9};
  int grid_tilt_steps{9};

  // Simple lookahead: evaluate clearance also at s + lookahead_s_m, and constrain/optimize
  // against the worst-case over {now, ahead}. This helps avoid stalling at the doorway.
  double lookahead_s_m{0.0};

  // Cost weights
  double w_center{8.0};
  double w_dl{2.0};
  double w_dt{2.0};
  double w_smooth{0.6};

  // Limits
  double base_lift_rate_limit_m_s{0.20};
  double base_tilt_rate_limit_rad_s{0.35};
  double base_speed_limit_m_s{1.0};

  // If geometrically feasible but tight, allow creeping forward.
  // Applied after degraded and pitch-rate factors.
  double min_speed_limit_m_s{0.02};

  // Degraded thresholds
  double pitch_rate_jitter_threshold_rad_s{0.45};

  // Degraded multipliers
  double degraded_margin_multiplier{2.0};
  double degraded_rate_multiplier{0.5};
  double degraded_speed_multiplier{0.5};
};

}  // namespace tlf

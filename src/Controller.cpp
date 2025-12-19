#include "controller/Controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace tlf {

static bool finiteAll(const ControlInput& in) {
  auto finite = [](double v) { return std::isfinite(v); };
  return finite(in.dt_s) && finite(in.pitch_rad) && finite(in.pitch_rate_rad_s) && finite(in.s_m) &&
         finite(in.lift_pos_m) && finite(in.tilt_rad) && finite(in.rack.height_m) && finite(in.rack.length_m) &&
         finite(in.rack.mount_offset_m.x) && finite(in.rack.mount_offset_m.z);
}

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

static double lerp(double a, double b, double t) { return a + (b - a) * t; }

static SafetyStatus makeSafety(const ControllerConfig& cfg,
                              double clearance_top_m,
                              double clearance_bottom_m,
                              CornerId worst,
                              bool degraded,
                              SafetyCode code_override = SafetyCode::None,
                              std::string message_override = {}) {
  SafetyStatus s;
  s.clearance_top_m = clearance_top_m;
  s.clearance_bottom_m = clearance_bottom_m;
  s.worst_point = worst;

  if (degraded) {
    s.level = SafetyLevel::DEGRADED;
    s.code = (code_override == SafetyCode::None) ? SafetyCode::InputInvalid : code_override;
    s.message = message_override.empty() ? "DEGRADED" : message_override;
    return s;
  }

  const double min_clear = std::min(clearance_top_m, clearance_bottom_m);

  if (min_clear < cfg.hard_threshold_m) {
    s.level = SafetyLevel::STOP;
    s.code = (code_override == SafetyCode::None) ? SafetyCode::ClearanceHardViolated : code_override;
    s.message = message_override.empty() ? "STOP: hard clearance violated" : message_override;
    return s;
  }

  if (min_clear < cfg.warn_threshold_m) {
    s.level = SafetyLevel::WARN;
    s.code = (code_override == SafetyCode::None) ? SafetyCode::ClearanceSoftNear : code_override;
    s.message = message_override.empty() ? "WARN: clearance near boundary" : message_override;
    return s;
  }

  s.level = SafetyLevel::OK;
  s.code = SafetyCode::None;
  s.message = "OK";

  // Allow non-fatal diagnostic codes even when geometrically OK.
  if (code_override != SafetyCode::None) {
    s.code = code_override;
    if (!message_override.empty()) s.message = message_override;
  }
  return s;
}

Controller::Controller(ControllerConfig cfg) : cfg_(cfg) {}

void Controller::reset() {
  time_s_ = 0.0;
  prev_lift_rate_m_s_ = 0.0;
  prev_tilt_rate_rad_s_ = 0.0;
}

DebugFrame Controller::step(const ControlInput& in) {
  DebugFrame f;
  f.in = in;

  const double dt = (in.dt_s > 1e-6 && std::isfinite(in.dt_s)) ? in.dt_s : 0.02;
  time_s_ += dt;
  f.time_s = time_s_;

  bool degraded = false;
  SafetyCode degraded_code = SafetyCode::None;
  std::string degraded_msg;

  if (!in.inputs_valid || !finiteAll(in) || !(dt > 0.0)) {
    degraded = true;
    degraded_code = SafetyCode::InputInvalid;
    degraded_msg = "Invalid inputs";
  } else if (std::abs(in.pitch_rate_rad_s) > cfg_.pitch_rate_jitter_threshold_rad_s) {
    degraded = true;
    degraded_code = SafetyCode::PitchJitter;
    degraded_msg = "Pitch rate jitter";
  }

  // Apply degraded multipliers
  const double margin_mult = degraded ? cfg_.degraded_margin_multiplier : 1.0;
  const double rate_mult = degraded ? cfg_.degraded_rate_multiplier : 1.0;
  const double speed_mult = degraded ? cfg_.degraded_speed_multiplier : 1.0;

  const double margin_top = cfg_.margin_top_m * margin_mult;
  const double margin_bottom = cfg_.margin_bottom_m * margin_mult;

  const double lift_rate_limit = cfg_.base_lift_rate_limit_m_s * rate_mult;
  const double tilt_rate_limit = cfg_.base_tilt_rate_limit_rad_s * rate_mult;

  // Current geometry
  f.corners = computeRackCorners2D(in.s_m, in.lift_pos_m, in.pitch_rad, in.tilt_rad, in.env, in.rack, in.forklift);
  const auto current_clear = computeClearances(f.corners, in.env, margin_top, margin_bottom);

  const double s_look = in.s_m + std::max(0.0, cfg_.lookahead_s_m);
  const auto current_clear_ahead = (cfg_.lookahead_s_m > 1e-9)
                                      ? computeClearances(computeRackCorners2D(s_look, in.lift_pos_m, in.pitch_rad, in.tilt_rad, in.env, in.rack, in.forklift),
                                                         in.env, margin_top, margin_bottom)
                                      : current_clear;
  const double current_clear_top_worst = std::min(current_clear.clearance_top_m, current_clear_ahead.clearance_top_m);
  const double current_clear_bottom_worst = std::min(current_clear.clearance_bottom_m, current_clear_ahead.clearance_bottom_m);

  // Search candidates
  const int nL = std::max(3, cfg_.grid_lift_steps);
  const int nT = std::max(3, cfg_.grid_tilt_steps);

  const double lift0 = in.lift_pos_m;
  const double tilt0 = in.tilt_rad;

  const double Lmin = lift0 - cfg_.search_lift_half_range_m;
  const double Lmax = lift0 + cfg_.search_lift_half_range_m;

  const double Tmin = tilt0 - cfg_.search_tilt_half_range_rad;
  const double Tmax = tilt0 + cfg_.search_tilt_half_range_rad;

  struct Best {
    bool feasible;
    double cost;
    double lift;
    double tilt;
    ClearanceResult clr;
  } best;

  best.feasible = false;
  best.cost = std::numeric_limits<double>::infinity();
  best.lift = lift0;
  best.tilt = tilt0;
  best.clr = current_clear;

  // If no feasible candidates exist, pick one that maximizes min clearance.
  double best_min_clear = -std::numeric_limits<double>::infinity();
  double best_min_lift = lift0;
  double best_min_tilt = tilt0;
  ClearanceResult best_min_clr = current_clear;

  for (int i = 0; i < nL; ++i) {
    const double tL = (nL == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(nL - 1);
    const double lift_c = lerp(Lmin, Lmax, tL);

    for (int j = 0; j < nT; ++j) {
      const double tT = (nT == 1) ? 0.0 : static_cast<double>(j) / static_cast<double>(nT - 1);
      const double tilt_c = lerp(Tmin, Tmax, tT);

      const auto corners = computeRackCorners2D(in.s_m, lift_c, in.pitch_rad, tilt_c, in.env, in.rack, in.forklift);
      const auto clr = computeClearances(corners, in.env, margin_top, margin_bottom);

      ClearanceResult clr_worst = clr;
      if (cfg_.lookahead_s_m > 1e-9) {
        const auto corners_a = computeRackCorners2D(s_look, lift_c, in.pitch_rad, tilt_c, in.env, in.rack, in.forklift);
        const auto clr_a = computeClearances(corners_a, in.env, margin_top, margin_bottom);
        clr_worst.clearance_top_m = std::min(clr.clearance_top_m, clr_a.clearance_top_m);
        clr_worst.clearance_bottom_m = std::min(clr.clearance_bottom_m, clr_a.clearance_bottom_m);
        clr_worst.worst_point = (clr_worst.clearance_top_m < clr_worst.clearance_bottom_m) ? clr.worst_point : clr.worst_point;
      }

      const double min_clear = std::min(clr_worst.clearance_top_m, clr_worst.clearance_bottom_m);
      if (min_clear > best_min_clear) {
        best_min_clear = min_clear;
        best_min_lift = lift_c;
        best_min_tilt = tilt_c;
        best_min_clr = clr_worst;
      }

      const bool feasible = (clr_worst.clearance_top_m >= 0.0) && (clr_worst.clearance_bottom_m >= 0.0);
      if (!feasible) continue;

      // Centering: clearance_mid = top - bottom, target is 0
      const double clearance_mid = clr_worst.clearance_top_m - clr_worst.clearance_bottom_m;

      const double lift_rate = (lift_c - lift0) / dt;
      const double tilt_rate = (tilt_c - tilt0) / dt;
      const double d_lift_rate = lift_rate - prev_lift_rate_m_s_;
      const double d_tilt_rate = tilt_rate - prev_tilt_rate_rad_s_;

      const double cost =
          cfg_.w_center * (clearance_mid * clearance_mid) +
          cfg_.w_dl * ((lift_c - lift0) * (lift_c - lift0)) +
          cfg_.w_dt * ((tilt_c - tilt0) * (tilt_c - tilt0)) +
          cfg_.w_smooth * (d_lift_rate * d_lift_rate + d_tilt_rate * d_tilt_rate);

      if (cost < best.cost) {
        best.feasible = true;
        best.cost = cost;
        best.lift = lift_c;
        best.tilt = tilt_c;
        best.clr = clr_worst;
      }
    }
  }

  double lift_star = lift0;
  double tilt_star = tilt0;
  ClearanceResult star_clr = current_clear;
  bool had_feasible = false;

  SafetyCode search_code = SafetyCode::None;
  std::string search_msg;

  if (best.feasible) {
    lift_star = best.lift;
    tilt_star = best.tilt;
    star_clr = best.clr;
    had_feasible = true;
  } else {
    lift_star = best_min_lift;
    tilt_star = best_min_tilt;
    star_clr = best_min_clr;
    had_feasible = false;
    search_code = SafetyCode::NoFeasibleSolution;
    search_msg = "No feasible (lift,tilt) in neighborhood";
  }

  // Compose command: targets are positions, rate limits are provided.
  f.cmd.lift_target_m = lift_star;
  f.cmd.tilt_target_rad = tilt_star;
  f.cmd.lift_rate_limit_m_s = lift_rate_limit;
  f.cmd.tilt_rate_limit_rad_s = tilt_rate_limit;

  // Simple speed policy: reduce as min clearance approaches 0, and when pitch_rate is high.
  const double min_clear = std::min(current_clear_top_worst, current_clear_bottom_worst);
  const double clearance_factor = clamp(min_clear / cfg_.warn_threshold_m, 0.0, 1.0);
  const double pitch_rate_factor = clamp(1.0 - (std::abs(in.pitch_rate_rad_s) / (2.0 * cfg_.pitch_rate_jitter_threshold_rad_s)), 0.2, 1.0);
  const double base_speed = cfg_.base_speed_limit_m_s * speed_mult;
  double speed = base_speed * std::min(clearance_factor, pitch_rate_factor);
  if (min_clear >= cfg_.hard_threshold_m) {
    speed = std::max(speed, cfg_.min_speed_limit_m_s * speed_mult * pitch_rate_factor);
  } else {
    speed = 0.0;
  }
  f.cmd.speed_limit_m_s = speed;

  f.had_feasible_solution = had_feasible;
  f.selected_cost = best.feasible ? best.cost : 0.0;

  // Safety status
  if (degraded) {
    f.safety = makeSafety(cfg_, current_clear_top_worst, current_clear_bottom_worst, current_clear.worst_point,
                          true, degraded_code, degraded_msg);
  } else {
    SafetyCode code = (search_code != SafetyCode::None) ? search_code : SafetyCode::None;
    std::string msg = (!search_msg.empty()) ? search_msg : std::string{};
    f.safety = makeSafety(cfg_, current_clear_top_worst, current_clear_bottom_worst, current_clear.worst_point,
                          false, code, msg);
  }

  // Update smoothing memory based on selected target (even if infeasible: still stabilize).
  prev_lift_rate_m_s_ = clamp((lift_star - lift0) / dt, -lift_rate_limit, lift_rate_limit);
  prev_tilt_rate_rad_s_ = clamp((tilt_star - tilt0) / dt, -tilt_rate_limit, tilt_rate_limit);

  return f;
}

}  // namespace tlf

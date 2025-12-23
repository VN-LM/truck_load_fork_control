#include "controller/ControllerMPC.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "model/Geometry.hpp"

namespace tlf {

static constexpr double kClearanceEpsilonM = 5e-4;

static bool finiteAll(const ControlInput& in) {
  auto finite = [](double v) { return std::isfinite(v); };
  return finite(in.dt_s) && finite(in.pitch_rad) && finite(in.pitch_rate_rad_s) && finite(in.s_m) &&
         finite(in.lift_pos_m) && finite(in.tilt_rad) && finite(in.rack.height_m) && finite(in.rack.length_m) &&
         finite(in.rack.mount_offset_m.x) && finite(in.rack.mount_offset_m.z);
}

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

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
  if (min_clear < (cfg.hard_threshold_m - kClearanceEpsilonM)) {
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

  if (code_override != SafetyCode::None) {
    s.code = code_override;
    if (!message_override.empty()) s.message = message_override;
  }
  return s;
}

ControllerMPC::ControllerMPC(ControllerConfig cfg) : cfg_(cfg) {}

void ControllerMPC::reset() {
  time_s_ = 0.0;
  prev_lift_rate_m_s_ = 0.0;
  prev_tilt_rate_rad_s_ = 0.0;
}

struct SeqNode {
  double cost{0.0};
  // predicted state
  double s_m{0.0};
  double lift_m{0.0};
  double tilt_rad{0.0};
  double last_lift_rate{0.0};
  double last_tilt_rate{0.0};
  // first action (to output)
  double u0_lift_rate{0.0};
  double u0_tilt_rate{0.0};
  bool has_u0{false};
};

DebugFrame ControllerMPC::step(const ControlInput& in) {
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

  // Optional: preserve existing single-step lookahead semantics for safety/speed reporting.
  const double s_look = in.s_m + std::max(0.0, cfg_.lookahead_s_m);
  const auto current_clear_ahead = (cfg_.lookahead_s_m > 1e-9)
                                      ? computeClearances(
                                            computeRackCorners2D(s_look, in.lift_pos_m, in.pitch_rad, in.tilt_rad, in.env, in.rack, in.forklift),
                                            in.env, margin_top, margin_bottom)
                                      : current_clear;

  const double current_clear_top_worst = std::min(current_clear.clearance_top_m, current_clear_ahead.clearance_top_m);
  const double current_clear_bottom_worst = std::min(current_clear.clearance_bottom_m, current_clear_ahead.clearance_bottom_m);

  // MPC/beam-search parameters
  const int H = std::max(1, cfg_.mpc_horizon_steps);
  const int beam = std::max(5, cfg_.mpc_beam_width);

  // Action set: a small discrete set of rate commands.
  // Keep it compact to stay real-time friendly.
  const double a1 = 1.0;
  const double a2 = 0.5;
  const double lift_rates[] = {-a1 * lift_rate_limit, -a2 * lift_rate_limit, 0.0, a2 * lift_rate_limit, a1 * lift_rate_limit};
  const double tilt_rates[] = {-a1 * tilt_rate_limit, -a2 * tilt_rate_limit, 0.0, a2 * tilt_rate_limit, a1 * tilt_rate_limit};

  const double lift0 = in.lift_pos_m;
  const double tilt0 = in.tilt_rad;

  // Predict forward progress (s). If 0, keep s constant.
  const double assumed_v = std::max(0.0, cfg_.mpc_assumed_forward_speed_m_s) * speed_mult;

  auto pitchAtStep = [&](int k) {
    if (cfg_.mpc_use_pitch_rate_prediction <= 0.0) return in.pitch_rad;
    return in.pitch_rad + in.pitch_rate_rad_s * dt * static_cast<double>(k);
  };

  auto stageCost = [&](double clearance_top_m, double clearance_bottom_m, double lift_m, double tilt_rad, double lift_rate, double tilt_rate, double prev_lift_rate, double prev_tilt_rate) {
    const double mid = clearance_top_m - clearance_bottom_m;
    const double d_lift_rate = lift_rate - prev_lift_rate;
    const double d_tilt_rate = tilt_rate - prev_tilt_rate;

    // Same spirit as single-step controller, applied per stage.
    const double cost_center = cfg_.w_center * (mid * mid);
    const double cost_mag = cfg_.w_dl * ((lift_m - lift0) * (lift_m - lift0)) + cfg_.w_dt * ((tilt_rad - tilt0) * (tilt_rad - tilt0));
    const double cost_smooth = cfg_.w_smooth * (d_lift_rate * d_lift_rate + d_tilt_rate * d_tilt_rate);

    return cost_center + cost_mag + cost_smooth;
  };

  // Beam search over sequences of rate commands.
  std::vector<SeqNode> frontier;
  frontier.reserve(static_cast<size_t>(beam));
  frontier.push_back(SeqNode{0.0, in.s_m, lift0, tilt0, prev_lift_rate_m_s_, prev_tilt_rate_rad_s_, 0.0, 0.0, false});

  bool any_feasible_sequence = false;
  SeqNode best_node;
  best_node.cost = std::numeric_limits<double>::infinity();

  for (int k = 0; k < H; ++k) {
    std::vector<SeqNode> next;
    next.reserve(static_cast<size_t>(beam) * 5);

    for (const auto& node : frontier) {
      for (double lr : lift_rates) {
        for (double tr : tilt_rates) {
          SeqNode child = node;

          // Apply dynamics
          const double lift_next = child.lift_m + lr * dt;
          const double tilt_next = child.tilt_rad + tr * dt;
          const double s_next = child.s_m + assumed_v * dt;

          const double pitch_k = pitchAtStep(k + 1);

          // Check constraints at the next predicted state
          const auto corners = computeRackCorners2D(s_next, lift_next, pitch_k, tilt_next, in.env, in.rack, in.forklift);
          const auto clr = computeClearances(corners, in.env, margin_top, margin_bottom);

          if (!(clr.clearance_top_m >= 0.0) || !(clr.clearance_bottom_m >= 0.0)) {
            continue;  // hard prune
          }

          // Optional spatial lookahead at s+lookahead (same tilt/lift), making it slightly more conservative.
          if (cfg_.lookahead_s_m > 1e-9) {
            const double s_a = s_next + cfg_.lookahead_s_m;
            const auto corners_a = computeRackCorners2D(s_a, lift_next, pitch_k, tilt_next, in.env, in.rack, in.forklift);
            const auto clr_a = computeClearances(corners_a, in.env, margin_top, margin_bottom);
            const double top_w = std::min(clr.clearance_top_m, clr_a.clearance_top_m);
            const double bot_w = std::min(clr.clearance_bottom_m, clr_a.clearance_bottom_m);
            if (!(top_w >= 0.0) || !(bot_w >= 0.0)) continue;

            const double cost = stageCost(top_w, bot_w, lift_next, tilt_next, lr, tr, child.last_lift_rate, child.last_tilt_rate);
            child.cost += cost;
          } else {
            const double cost = stageCost(clr.clearance_top_m, clr.clearance_bottom_m, lift_next, tilt_next, lr, tr, child.last_lift_rate, child.last_tilt_rate);
            child.cost += cost;
          }

          child.s_m = s_next;
          child.lift_m = lift_next;
          child.tilt_rad = tilt_next;
          child.last_lift_rate = lr;
          child.last_tilt_rate = tr;

          if (!child.has_u0) {
            child.u0_lift_rate = lr;
            child.u0_tilt_rate = tr;
            child.has_u0 = true;
          }

          next.push_back(child);
        }
      }
    }

    if (next.empty()) {
      break;
    }

    // Keep best beam candidates
    std::nth_element(next.begin(),
                     next.begin() + std::min(static_cast<int>(next.size()), beam) - 1,
                     next.end(),
                     [](const SeqNode& a, const SeqNode& b) { return a.cost < b.cost; });
    if (static_cast<int>(next.size()) > beam) {
      next.resize(static_cast<size_t>(beam));
    }

    frontier = std::move(next);
  }

  // Pick best sequence in frontier
  for (const auto& node : frontier) {
    any_feasible_sequence = true;
    if (node.cost < best_node.cost) best_node = node;
  }

  double lift_star = lift0;
  double tilt_star = tilt0;
  bool had_feasible = false;

  SafetyCode search_code = SafetyCode::None;
  std::string search_msg;

  if (any_feasible_sequence && best_node.has_u0) {
    // Convert first rate action to a near-term target position.
    lift_star = lift0 + clamp(best_node.u0_lift_rate, -lift_rate_limit, lift_rate_limit) * dt;
    tilt_star = tilt0 + clamp(best_node.u0_tilt_rate, -tilt_rate_limit, tilt_rate_limit) * dt;
    had_feasible = true;
  } else {
    // Fallback: do a single-step best-effort search in the same neighborhood as the original controller.
    // (Keeps behavior safe even if MPC horizon becomes infeasible.)
    search_code = SafetyCode::NoFeasibleSolution;
    search_msg = "No feasible MPC sequence";

    const int nL = std::max(3, cfg_.grid_lift_steps);
    const int nT = std::max(3, cfg_.grid_tilt_steps);

    const double Lmin = lift0 - cfg_.search_lift_half_range_m;
    const double Lmax = lift0 + cfg_.search_lift_half_range_m;
    const double Tmin = tilt0 - cfg_.search_tilt_half_range_rad;
    const double Tmax = tilt0 + cfg_.search_tilt_half_range_rad;

    double best_min_clear = -std::numeric_limits<double>::infinity();
    double best_min_lift = lift0;
    double best_min_tilt = tilt0;

    for (int i = 0; i < nL; ++i) {
      const double tL = (nL == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(nL - 1);
      const double lift_c = Lmin + (Lmax - Lmin) * tL;
      for (int j = 0; j < nT; ++j) {
        const double tT = (nT == 1) ? 0.0 : static_cast<double>(j) / static_cast<double>(nT - 1);
        const double tilt_c = Tmin + (Tmax - Tmin) * tT;

        const auto corners = computeRackCorners2D(in.s_m, lift_c, in.pitch_rad, tilt_c, in.env, in.rack, in.forklift);
        const auto clr = computeClearances(corners, in.env, margin_top, margin_bottom);
        double top_w = clr.clearance_top_m;
        double bot_w = clr.clearance_bottom_m;
        if (cfg_.lookahead_s_m > 1e-9) {
          const auto corners_a = computeRackCorners2D(s_look, lift_c, in.pitch_rad, tilt_c, in.env, in.rack, in.forklift);
          const auto clr_a = computeClearances(corners_a, in.env, margin_top, margin_bottom);
          top_w = std::min(top_w, clr_a.clearance_top_m);
          bot_w = std::min(bot_w, clr_a.clearance_bottom_m);
        }

        const double min_clear = std::min(top_w, bot_w);
        if (min_clear > best_min_clear) {
          best_min_clear = min_clear;
          best_min_lift = lift_c;
          best_min_tilt = tilt_c;
        }
      }
    }

    lift_star = best_min_lift;
    tilt_star = best_min_tilt;
    had_feasible = false;
  }

  f.cmd.lift_target_m = lift_star;
  f.cmd.tilt_target_rad = tilt_star;
  f.cmd.lift_rate_limit_m_s = lift_rate_limit;
  f.cmd.tilt_rate_limit_rad_s = tilt_rate_limit;

  // Speed policy: same as original controller
  const double min_clear = std::min(current_clear_top_worst, current_clear_bottom_worst);
  const double clearance_factor = clamp(min_clear / cfg_.warn_threshold_m, 0.0, 1.0);
  const double pitch_rate_factor = clamp(1.0 - (std::abs(in.pitch_rate_rad_s) / (2.0 * cfg_.pitch_rate_jitter_threshold_rad_s)), 0.2, 1.0);
  const double base_speed = cfg_.base_speed_limit_m_s * speed_mult;
  double speed = base_speed * std::min(clearance_factor, pitch_rate_factor);
  if (min_clear >= (cfg_.hard_threshold_m - kClearanceEpsilonM)) {
    speed = std::max(speed, cfg_.min_speed_limit_m_s * speed_mult * pitch_rate_factor);
  } else {
    speed = 0.0;
  }
  f.cmd.speed_limit_m_s = speed;

  f.had_feasible_solution = had_feasible;
  f.selected_cost = any_feasible_sequence ? best_node.cost : 0.0;

  // Safety
  if (degraded) {
    f.safety = makeSafety(cfg_, current_clear_top_worst, current_clear_bottom_worst, current_clear.worst_point,
                          true, degraded_code, degraded_msg);
  } else {
    SafetyCode code = (search_code != SafetyCode::None) ? search_code : SafetyCode::None;
    std::string msg = search_msg;
    f.safety = makeSafety(cfg_, current_clear_top_worst, current_clear_bottom_worst, current_clear.worst_point,
                          false, code, msg);
  }

  // Update smoothing memory based on chosen near-term target.
  prev_lift_rate_m_s_ = clamp((lift_star - lift0) / dt, -lift_rate_limit, lift_rate_limit);
  prev_tilt_rate_rad_s_ = clamp((tilt_star - tilt0) / dt, -tilt_rate_limit, tilt_rate_limit);

  return f;
}

}  // namespace tlf

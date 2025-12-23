#pragma once

#include "controller/IController.hpp"

namespace tlf {

// A lightweight, real-time friendly MPC-style controller using short-horizon beam search.
// It reuses the same geometry + clearance constraints as the grid-search controller.
class ControllerMPC final : public IController {
 public:
  explicit ControllerMPC(ControllerConfig cfg = {});

  const ControllerConfig& config() const override { return cfg_; }
  ControllerConfig& config() override { return cfg_; }

  DebugFrame step(const ControlInput& in) override;
  void reset() override;

 private:
  ControllerConfig cfg_;
  double time_s_{0.0};

  // smoothing memory (for cost regularization, not plant feedback)
  double prev_lift_rate_m_s_{0.0};
  double prev_tilt_rate_rad_s_{0.0};
};

}  // namespace tlf

#pragma once

#include "controller/Types.hpp"

namespace tlf {

class Controller {
 public:
  explicit Controller(ControllerConfig cfg = {});

  const ControllerConfig& config() const { return cfg_; }
  ControllerConfig& config() { return cfg_; }

  // Stateless from caller perspective; internal state is used only for smoothing.
  DebugFrame step(const ControlInput& in);

  void reset();

 private:
  ControllerConfig cfg_;
  double time_s_{0.0};

  // smoothing memory
  double prev_lift_rate_m_s_{0.0};
  double prev_tilt_rate_rad_s_{0.0};
};

}  // namespace tlf

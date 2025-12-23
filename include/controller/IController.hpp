#pragma once

#include "controller/Types.hpp"

namespace tlf {

class IController {
 public:
  virtual ~IController() = default;

  virtual const ControllerConfig& config() const = 0;
  virtual ControllerConfig& config() = 0;

  virtual DebugFrame step(const ControlInput& in) = 0;
  virtual void reset() = 0;
};

}  // namespace tlf

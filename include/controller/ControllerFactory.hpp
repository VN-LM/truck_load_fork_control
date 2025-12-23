#pragma once

#include <cctype>
#include <memory>
#include <string>

#include "controller/IController.hpp"
#include "controller/Controller.hpp"
#include "controller/ControllerMPC.hpp"

namespace tlf {

enum class ControllerKind : int {
  GridSearch = 0,
  MPC = 1,
};

inline const char* toString(ControllerKind k) {
  switch (k) {
    case ControllerKind::GridSearch:
      return "grid";
    case ControllerKind::MPC:
      return "mpc";
    default:
      return "unknown";
  }
}

inline std::unique_ptr<IController> makeController(ControllerKind kind, const ControllerConfig& cfg) {
  switch (kind) {
    case ControllerKind::MPC:
      return std::make_unique<ControllerMPC>(cfg);
    case ControllerKind::GridSearch:
    default:
      return std::make_unique<Controller>(cfg);
  }
}

inline ControllerKind controllerKindFromString(std::string s) {
  for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  if (s == "mpc") return ControllerKind::MPC;
  return ControllerKind::GridSearch;
}

}  // namespace tlf

#pragma once

#include <fstream>
#include <string>

#include "controller/Types.hpp"

namespace tlf {

class CsvLogger {
 public:
  explicit CsvLogger(std::string path);
  bool good() const { return out_.good(); }

  void writeHeader();
  void writeFrame(const DebugFrame& f);

 private:
  std::ofstream out_;
};

}  // namespace tlf

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "controller/Controller.hpp"

using namespace tlf;

static std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : line) {
    if (ch == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(ch);
    }
  }
  out.push_back(cur);
  return out;
}

int main(int argc, char** argv) {
  std::string path;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--log" && i + 1 < argc) path = argv[++i];
  }
  if (path.empty()) {
    std::cerr << "Usage: example_log_replay --log <csv>\n";
    return 2;
  }

  std::ifstream in(path);
  if (!in.good()) {
    std::cerr << "Failed to open: " << path << "\n";
    return 1;
  }

  std::string header;
  std::getline(in, header);

  int count = 0;
  double min_top = 1e9;
  double min_bottom = 1e9;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsvLine(line);
    if (fields.size() < 10) continue;

    // clearance_top at index 16, clearance_bottom at 17 per docs/log_format.md (after 8 corner fields)
    // Here we keep it robust: search near end.
    const double clearance_top = std::stod(fields[16]);
    const double clearance_bottom = std::stod(fields[17]);

    min_top = std::min(min_top, clearance_top);
    min_bottom = std::min(min_bottom, clearance_bottom);
    ++count;
  }

  std::cout << "Frames: " << count << "\n";
  std::cout << "Min clearance_top: " << min_top << " m\n";
  std::cout << "Min clearance_bottom: " << min_bottom << " m\n";
  return 0;
}

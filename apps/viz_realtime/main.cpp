#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "controller/Controller.hpp"
#include "controller/ControllerFactory.hpp"
#include "model/Geometry.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include <GLFW/glfw3.h>

using namespace tlf;

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

struct VizSample {
  double time_s{0.0};
  double s_m{0.0};
  double pitch_rad{0.0};
  double pitch_rate_rad_s{0.0};
  double lift_m{0.0};
  double tilt_rad{0.0};
  double ceiling_z{2.5};
  double floor_z{0.0};

  CornerPoints2D corners;
  double clearance_top{0.0};
  double clearance_bottom{0.0};

  double lift_cmd{0.0};
  double tilt_cmd{0.0};
  double speed_limit{1.0};

  int safety_level{0};
  int terrain_state{0};
  int worst_point{0};
};

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

static bool loadCsvLog(const std::string& path, std::vector<VizSample>* out) {
  out->clear();
  std::ifstream in(path);
  if (!in.good()) return false;

  std::string header;
  if (!std::getline(in, header)) return false;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto f = splitCsvLine(line);
    // Expect MVP header fields count
    if (f.size() < 26) continue;

    VizSample s;
    s.time_s = std::stod(f[0]);
    s.s_m = std::stod(f[1]);
    s.pitch_rad = std::stod(f[2]);
    s.pitch_rate_rad_s = std::stod(f[3]);
    s.lift_m = std::stod(f[4]);
    s.tilt_rad = std::stod(f[5]);
    s.ceiling_z = std::stod(f[6]);
    s.floor_z = std::stod(f[7]);

    s.corners.p[0] = {std::stod(f[8]), std::stod(f[9])};
    s.corners.p[1] = {std::stod(f[10]), std::stod(f[11])};
    s.corners.p[2] = {std::stod(f[12]), std::stod(f[13])};
    s.corners.p[3] = {std::stod(f[14]), std::stod(f[15])};

    s.clearance_top = std::stod(f[16]);
    s.clearance_bottom = std::stod(f[17]);

    s.lift_cmd = std::stod(f[18]);
    s.tilt_cmd = std::stod(f[19]);
    s.speed_limit = std::stod(f[20]);

    s.safety_level = std::stoi(f[21]);
    s.terrain_state = std::stoi(f[22]);
    s.worst_point = std::stoi(f[23]);

    out->push_back(s);
  }

  return !out->empty();
}

static double rampFloorZ(double x_m, double ramp_deg) {
  const double a = std::tan(ramp_deg * M_PI / 180.0);
  if (x_m < 0.0) return a * x_m;  // outside is lower
  return 0.0;
}

static TerrainState terrainFromS(double s) {
  if (s < -1.2) return TerrainState::Ground;
  if (s < -0.6) return TerrainState::FrontOnRamp;
  if (s < -0.1) return TerrainState::OnRamp;
  if (s < 0.5) return TerrainState::FrontInContainerRearOnRamp;
  return TerrainState::InContainer;
}

static double pitchFromTerrain(TerrainState t, double s) {
  const double ramp = 4.0 * M_PI / 180.0;
  switch (t) {
    case TerrainState::Ground:
      return 0.0;
    case TerrainState::FrontOnRamp:
    case TerrainState::OnRamp:
      return ramp;
    case TerrainState::FrontInContainerRearOnRamp: {
      const double t01 = clamp((s - (-0.1)) / (0.6), 0.0, 1.0);
      return (1.0 - t01) * ramp;
    }
    case TerrainState::InContainer:
    default:
      return 0.0;
  }
}

static std::vector<VizSample> buildBuiltinTrajectory(const ControllerConfig& cfg, ControllerKind kind, int steps = 900) {
  auto controller = makeController(kind, cfg);
  controller->reset();

  RackParams rack;
  rack.height_m = 2.3;
  rack.length_m = 2.3;
  rack.mount_offset_m = {0.3, -0.15};

  ForkliftParams fl;

  double time_s = 0.0;
  double s_m = -1.5;
  double lift_m = 1.00;
  double tilt_rad = 0.0;
  double last_pitch = 0.0;

  const double dt = 0.02;
  const double v = 0.35;

  std::vector<VizSample> out;
  out.reserve(static_cast<size_t>(steps));

  for (int k = 0; k < steps; ++k) {
    const TerrainState terr = terrainFromS(s_m);
    const double pitch = pitchFromTerrain(terr, s_m);
    const double pitch_rate = (pitch - last_pitch) / dt;

    EnvironmentGeometry env;
    env.ceiling_z_m = 2.5;
    env.floor_z_m = rampFloorZ(s_m, 4.0);

    ControlInput in;
    in.dt_s = dt;
    in.s_m = s_m;
    in.pitch_rad = pitch;
    in.pitch_rate_rad_s = pitch_rate;
    in.terrain = terr;
    in.lift_pos_m = lift_m;
    in.tilt_rad = tilt_rad;
    in.env = env;
    in.rack = rack;
    in.forklift = fl;
    in.inputs_valid = true;

    const DebugFrame fr = controller->step(in);

    VizSample vs;
    vs.time_s = fr.time_s;
    vs.s_m = in.s_m;
    vs.pitch_rad = in.pitch_rad;
    vs.pitch_rate_rad_s = in.pitch_rate_rad_s;
    vs.lift_m = in.lift_pos_m;
    vs.tilt_rad = in.tilt_rad;
    vs.ceiling_z = env.ceiling_z_m.value();
    vs.floor_z = env.floor_z_m.value();
    vs.corners = fr.corners;
    vs.clearance_top = fr.safety.clearance_top_m;
    vs.clearance_bottom = fr.safety.clearance_bottom_m;
    vs.lift_cmd = fr.cmd.lift_target_m;
    vs.tilt_cmd = fr.cmd.tilt_target_rad;
    vs.speed_limit = fr.cmd.speed_limit_m_s;
    vs.safety_level = static_cast<int>(fr.safety.level);
    vs.terrain_state = static_cast<int>(in.terrain);
    vs.worst_point = static_cast<int>(fr.safety.worst_point);

    out.push_back(vs);

    // apply actuation
    const double lift_err = fr.cmd.lift_target_m - lift_m;
    lift_m += clamp(lift_err, -fr.cmd.lift_rate_limit_m_s * dt, fr.cmd.lift_rate_limit_m_s * dt);

    const double tilt_err = fr.cmd.tilt_target_rad - tilt_rad;
    tilt_rad += clamp(tilt_err, -fr.cmd.tilt_rate_limit_rad_s * dt, fr.cmd.tilt_rate_limit_rad_s * dt);

    const double speed = std::min(v, fr.cmd.speed_limit_m_s);
    s_m += speed * dt;

    time_s += dt;
    last_pitch = pitch;

    if (s_m > 1.6) break;
  }

  return out;
}

static ImU32 colorForSafety(int level) {
  switch (level) {
    case 0:
      return IM_COL32(80, 200, 120, 255);
    case 1:
      return IM_COL32(240, 200, 80, 255);
    case 2:
      return IM_COL32(240, 80, 80, 255);
    case 3:
    default:
      return IM_COL32(160, 160, 220, 255);
  }
}

static void drawScene2D(const VizSample& s, const ImVec2& canvas_pos, const ImVec2& canvas_size) {
  ImDrawList* dl = ImGui::GetWindowDrawList();

  // World bounds for MVP
  const double x_min = -2.0;
  const double x_max = 2.2;
  const double z_min = -0.8;
  const double z_max = 3.0;

  auto W2S = [&](double x, double z) {
    const float u = static_cast<float>((x - x_min) / (x_max - x_min));
    const float v = static_cast<float>((z - z_min) / (z_max - z_min));
    const float sx = canvas_pos.x + u * canvas_size.x;
    const float sy = canvas_pos.y + (1.0f - v) * canvas_size.y;
    return ImVec2(sx, sy);
  };

  // Background
  dl->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(25, 25, 28, 255));
  dl->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(80, 80, 90, 255));

  // Door frame at x=0
  dl->AddLine(W2S(0.0, 0.0), W2S(0.0, s.ceiling_z), IM_COL32(200, 200, 220, 255), 2.0f);

  // Floor: ramp outside + container inside
  const double ramp_deg = 4.0;
  auto floorZ = [&](double x) { return rampFloorZ(x, ramp_deg); };

  // sample floor polyline
  const int N = 60;
  for (int i = 0; i < N - 1; ++i) {
    const double xa = x_min + (x_max - x_min) * (static_cast<double>(i) / (N - 1));
    const double xb = x_min + (x_max - x_min) * (static_cast<double>(i + 1) / (N - 1));
    dl->AddLine(W2S(xa, floorZ(xa)), W2S(xb, floorZ(xb)), IM_COL32(140, 140, 160, 255), 2.0f);
  }

  // Ceiling line (draw constant for MVP)
  dl->AddLine(W2S(x_min, s.ceiling_z), W2S(x_max, s.ceiling_z), IM_COL32(120, 120, 140, 255), 1.0f);

  // Rack rectangle
  const ImU32 rack_col = colorForSafety(s.safety_level);
  const auto& p = s.corners.p;
  dl->AddLine(W2S(p[0].x, p[0].z), W2S(p[2].x, p[2].z), rack_col, 3.0f);
  dl->AddLine(W2S(p[2].x, p[2].z), W2S(p[3].x, p[3].z), rack_col, 3.0f);
  dl->AddLine(W2S(p[3].x, p[3].z), W2S(p[1].x, p[1].z), rack_col, 3.0f);
  dl->AddLine(W2S(p[1].x, p[1].z), W2S(p[0].x, p[0].z), rack_col, 3.0f);

  // Simple fork pivot marker at (s, lift)
  dl->AddCircleFilled(W2S(s.s_m, s.lift_m), 4.0f, IM_COL32(220, 220, 220, 255));
}

int main(int argc, char** argv) {
  std::string log_path;
  ControllerKind controller_kind = ControllerKind::GridSearch;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--log" && i + 1 < argc) log_path = argv[++i];
    if (std::string(argv[i]) == "--controller" && i + 1 < argc) controller_kind = controllerKindFromString(argv[++i]);
  }

  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to init GLFW\n");
    return 1;
  }

  GLFWwindow* window = glfwCreateWindow(1280, 720, "tlf viz_realtime", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

  ControllerConfig cfg;
  std::vector<VizSample> samples;

  enum class Mode { Builtin, Log };
  Mode mode = log_path.empty() ? Mode::Builtin : Mode::Log;

  char log_path_buf[512] = {0};
  if (!log_path.empty()) {
    std::snprintf(log_path_buf, sizeof(log_path_buf), "%s", log_path.c_str());
  }

  auto rebuild = [&]() {
    if (mode == Mode::Builtin) {
      samples = buildBuiltinTrajectory(cfg, controller_kind);
    } else {
      std::vector<VizSample> tmp;
      if (loadCsvLog(std::string(log_path_buf), &tmp)) samples = std::move(tmp);
    }
  };

  rebuild();

  bool playing = true;
  int idx = 0;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize(ImVec2(1200, 680), ImGuiCond_FirstUseEver);
    ImGui::Begin("Realtime 2D Debug");

    // Controls
    if (ImGui::Button(playing ? "Pause" : "Play")) playing = !playing;
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
      playing = false;
      idx = std::min(idx + 1, static_cast<int>(samples.size()) - 1);
    }

    ImGui::SameLine();
    const char* mode_items[] = {"Builtin trajectory", "Log replay"};
    int mode_i = (mode == Mode::Builtin) ? 0 : 1;
    if (ImGui::Combo("Mode", &mode_i, mode_items, 2)) {
      mode = (mode_i == 0) ? Mode::Builtin : Mode::Log;
      idx = 0;
      rebuild();
    }

    if (mode == Mode::Log) {
      ImGui::InputText("CSV log path", log_path_buf, sizeof(log_path_buf));
      ImGui::SameLine();
      if (ImGui::Button("Load")) {
        idx = 0;
        rebuild();
      }
    }

    if (mode == Mode::Builtin) {
      const char* ctrl_items[] = {"Grid search", "MPC (beam search)"};
      int ctrl_i = (controller_kind == ControllerKind::GridSearch) ? 0 : 1;
      if (ImGui::Combo("Controller", &ctrl_i, ctrl_items, 2)) {
        controller_kind = (ctrl_i == 0) ? ControllerKind::GridSearch : ControllerKind::MPC;
        idx = 0;
        rebuild();
      }
    }

    if (samples.empty()) {
      ImGui::TextColored(ImVec4(1, 0.5f, 0.5f, 1), "No samples loaded.");
    } else {
      idx = std::max(0, std::min(idx, static_cast<int>(samples.size()) - 1));
      ImGui::SliderInt("Time", &idx, 0, static_cast<int>(samples.size()) - 1);

      const VizSample& cur = samples[static_cast<size_t>(idx)];

      // Scene
      const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
      const ImVec2 canvas_size = ImVec2(820, 520);
      ImGui::InvisibleButton("canvas", canvas_size);
      drawScene2D(cur, canvas_pos, canvas_size);

      ImGui::SameLine();
      ImGui::BeginGroup();

      ImGui::Text("t=%.2fs  s=%.2fm", cur.time_s, cur.s_m);
      ImGui::Text("pitch=%.2fdeg  pitch_rate=%.2fdeg/s", cur.pitch_rad * 180.0 / M_PI, cur.pitch_rate_rad_s * 180.0 / M_PI);
      ImGui::Text("lift=%.3fm  tilt=%.2fdeg", cur.lift_m, cur.tilt_rad * 180.0 / M_PI);
      ImGui::Separator();
      ImGui::Text("clear_top=%.3fm", cur.clearance_top);
      ImGui::Text("clear_bottom=%.3fm", cur.clearance_bottom);
      ImGui::Text("speed_limit=%.2fm/s", cur.speed_limit);
      ImGui::Separator();
      ImGui::Text("safety_level=%d  terrain=%d  worst=%d", cur.safety_level, cur.terrain_state, cur.worst_point);

      ImGui::Separator();
      ImGui::Text("Controller Params");
      bool changed = false;
      changed |= ImGui::SliderFloat("margin_top (m)", (float*)&cfg.margin_top_m, 0.0f, 0.30f);
      changed |= ImGui::SliderFloat("margin_bottom (m)", (float*)&cfg.margin_bottom_m, 0.0f, 0.30f);
      changed |= ImGui::SliderFloat("warn_threshold (m)", (float*)&cfg.warn_threshold_m, 0.01f, 0.50f);
      changed |= ImGui::SliderFloat("hard_threshold (m)", (float*)&cfg.hard_threshold_m, -0.05f, 0.10f);

      changed |= ImGui::SliderFloat("search_lift_half (m)", (float*)&cfg.search_lift_half_range_m, 0.02f, 0.40f);
      changed |= ImGui::SliderFloat("search_tilt_half (deg)", (float*)&cfg.search_tilt_half_range_rad, 0.01f, 0.40f);

      changed |= ImGui::SliderFloat("w_center", (float*)&cfg.w_center, 0.0f, 50.0f);
      changed |= ImGui::SliderFloat("w_dl", (float*)&cfg.w_dl, 0.0f, 20.0f);
      changed |= ImGui::SliderFloat("w_dt", (float*)&cfg.w_dt, 0.0f, 20.0f);
      changed |= ImGui::SliderFloat("w_smooth", (float*)&cfg.w_smooth, 0.0f, 5.0f);

      changed |= ImGui::SliderFloat("lift_rate_limit", (float*)&cfg.base_lift_rate_limit_m_s, 0.02f, 0.80f);
      changed |= ImGui::SliderFloat("tilt_rate_limit", (float*)&cfg.base_tilt_rate_limit_rad_s, 0.05f, 1.50f);
      changed |= ImGui::SliderFloat("base_speed_limit", (float*)&cfg.base_speed_limit_m_s, 0.05f, 2.00f);

      if (mode == Mode::Builtin && controller_kind == ControllerKind::MPC) {
        ImGui::Separator();
        ImGui::Text("MPC Params");
        changed |= ImGui::SliderInt("mpc_horizon_steps", &cfg.mpc_horizon_steps, 1, 12);
        changed |= ImGui::SliderInt("mpc_beam_width", &cfg.mpc_beam_width, 5, 120);
        changed |= ImGui::SliderFloat("mpc_assumed_forward_speed", (float*)&cfg.mpc_assumed_forward_speed_m_s, 0.0f, 1.5f);
        changed |= ImGui::SliderFloat("mpc_use_pitch_rate_pred", (float*)&cfg.mpc_use_pitch_rate_prediction, 0.0f, 1.0f);
      }

      if (changed && mode == Mode::Builtin) {
        const int keep = idx;
        rebuild();
        idx = std::min(keep, static_cast<int>(samples.size()) - 1);
      }

      ImGui::EndGroup();

      if (playing) {
        idx = std::min(idx + 1, static_cast<int>(samples.size()) - 1);
      }
    }

    ImGui::End();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.07f, 0.07f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

cmake_minimum_required(VERSION 3.20)

project(truck_load_fork_control VERSION 0.1.0 LANGUAGES CXX)

option(TLF_BUILD_VIZ "Build realtime ImGui visualization app" ON)
option(TLF_BUILD_EXAMPLES "Build examples" ON)
option(TLF_BUILD_TESTS "Build unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(truck_load_control)

target_sources(truck_load_control
  PRIVATE
    src/Controller.cpp
    src/ControllerMPC.cpp
    src/Geometry.cpp
    src/CsvLog.cpp
)

target_include_directories(truck_load_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(truck_load_control PUBLIC cxx_std_17)

target_compile_options(truck_load_control PRIVATE
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -Wpedantic>
  $<$<CXX_COMPILER_ID:AppleClang>:-Wall -Wextra -Wpedantic>
  $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wpedantic>
)

include(FetchContent)

# -------------------- Viz app (ImGui + GLFW + OpenGL2) --------------------
if(TLF_BUILD_VIZ)
  find_package(OpenGL REQUIRED)

  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.9
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw)

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.9
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui_static)
  target_sources(imgui_static PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
  )
  target_include_directories(imgui_static PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  target_link_libraries(imgui_static PUBLIC glfw)

  add_executable(viz_realtime
    apps/viz_realtime/main.cpp
  )
  target_link_libraries(viz_realtime
    PRIVATE
      truck_load_control
      imgui_static
      glfw
      OpenGL::GL
  )
  if(APPLE)
    target_compile_definitions(viz_realtime PRIVATE GL_SILENCE_DEPRECATION)
  endif()
  target_compile_definitions(viz_realtime PRIVATE IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
endif()

# -------------------- Examples --------------------
if(TLF_BUILD_EXAMPLES)
  add_executable(example_sim_trajectory examples/example_sim_trajectory.cpp)
  target_link_libraries(example_sim_trajectory PRIVATE truck_load_control)

  add_executable(example_log_replay examples/example_log_replay.cpp)
  target_link_libraries(example_log_replay PRIVATE truck_load_control)
endif()

# -------------------- Tests --------------------
if(TLF_BUILD_TESTS)
  include(CTest)
  enable_testing()

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
  )
  FetchContent_MakeAvailable(Catch2)

  add_executable(tlf_tests
    tests/test_geometry.cpp
    tests/test_search_and_safety.cpp
  )
  target_link_libraries(tlf_tests PRIVATE truck_load_control Catch2::Catch2WithMain)
  add_test(NAME tlf_tests COMMAND tlf_tests)
endif()

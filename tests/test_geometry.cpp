#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "model/Geometry.hpp"

using namespace tlf;

TEST_CASE("computeRackCorners2D basic shape") {
  RackParams rack;
  rack.height_m = 2.0;
  rack.length_m = 3.0;
  rack.mount_offset_m = {0.0, 0.0};

  ForkliftParams fl;
  fl.mast_pivot_height_m = 0.0;

  EnvironmentGeometry env;
  env.floor_z_m = 0.0;

  const auto c = computeRackCorners2D(/*s*/ 1.0, /*lift*/ 1.5, /*pitch*/ 0.0, /*tilt*/ 0.0, env, rack, fl);

  REQUIRE(c.p[static_cast<int>(CornerId::RearBottom)].x == Catch::Approx(1.0));
  REQUIRE(c.p[static_cast<int>(CornerId::RearBottom)].z == Catch::Approx(1.5));

  REQUIRE(c.p[static_cast<int>(CornerId::FrontBottom)].x == Catch::Approx(4.0));
  REQUIRE(c.p[static_cast<int>(CornerId::FrontBottom)].z == Catch::Approx(1.5));

  REQUIRE(c.p[static_cast<int>(CornerId::RearTop)].x == Catch::Approx(1.0));
  REQUIRE(c.p[static_cast<int>(CornerId::RearTop)].z == Catch::Approx(3.5));
}

TEST_CASE("computeClearances scalar env") {
  CornerPoints2D corners;
  corners.p[0] = {0.0, 0.2};
  corners.p[1] = {0.0, 2.2};
  corners.p[2] = {2.0, 0.2};
  corners.p[3] = {2.0, 2.2};

  EnvironmentGeometry env;
  env.floor_z_m = 0.0;
  env.ceiling_z_m = 2.5;

  auto clr = computeClearances(corners, env, /*margin_top*/ 0.1, /*margin_bottom*/ 0.1);
  REQUIRE(clr.clearance_top_m == Catch::Approx(0.2));  // 2.5-2.2-0.1
  REQUIRE(clr.clearance_bottom_m == Catch::Approx(0.1));  // 0.2-0.0-0.1
}

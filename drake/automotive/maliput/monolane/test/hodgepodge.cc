#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <iostream>

#include "gtest/gtest.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

GTEST_TEST(HodgePodge, Podge) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg = RoadGeometry({"apple"});
  Lane* l1 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {100., 100.}, {-5., 5.}, {-10., 10.}, zp, zp);
  EXPECT_NEAR(l1->length(), 100. * std::sqrt(2.), 1e-7);
  EXPECT_NEAR(l1->lane_bounds(0.).r_min_, -5., 1e-7);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max_,  5., 1e-7);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min_, -10., 1e-7);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max_,  10., 1e-7);

  xyz = l1->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 0., 1e-2);
  EXPECT_NEAR(xyz.y_, 0., 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100. * (1. / 141.42), 1e-2);
  EXPECT_NEAR(xyz.y_, 100. * (1. / 141.42), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({1., 1., 0.});
  EXPECT_NEAR(xyz.x_, 0, 1e-2);
  EXPECT_NEAR(xyz.y_, 2. * 100. * (1. / 141.42), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(xyz.x_, 100. * (-1. / 141.42), 1e-2);
  EXPECT_NEAR(xyz.y_, 100. * (1. / 141.42), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({141.42, 0., 0.});
  EXPECT_NEAR(xyz.x_, 100., 1e-2);
  EXPECT_NEAR(xyz.y_, 100., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  const double kPi = 3.14159;
  Lane* l2 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {200., 200.}, 100., 0., kPi / 2.,
      {-5., 5.}, {-10., 10.}, zp, zp);
  EXPECT_NEAR(l2->length(), 100. * kPi / 2., 1e-7);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 300., 1e-2);
  EXPECT_NEAR(xyz.y_, 200., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  xyz = l2->ToGeoPosition({100. * kPi / 2., 0., 0.});
  EXPECT_NEAR(xyz.x_, 200., 1e-2);
  EXPECT_NEAR(xyz.y_, 300., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  rot = l2->GetOrientation({100. * kPi / 2., 0., 0.});
  EXPECT_NEAR(rot.roll_,   0., 1e-2);
  EXPECT_NEAR(rot.pitch_,  0., 1e-2);
  EXPECT_NEAR(rot.yaw_,   kPi, 1e-2);


  Lane* l3 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewLineLane(
      {"l3"}, {0., 200.}, {0., -100.},
      {-5., 5.}, {-10., 10.},
      // elevation = (7 + 1.*p) * 100.
      //  [100. is the scale factor derived from xy projection of path.]
      {7., 1., 0., 0.},
      zp);
  EXPECT_NEAR(l3->length(), 100. * std::sqrt(2.), 1e-7);

  xyz = l3->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_,   0., 1e-2);
  EXPECT_NEAR(xyz.y_, 200., 1e-2);
  EXPECT_NEAR(xyz.z_, 700., 1e-2);

  xyz = l3->ToGeoPosition({50. * std::sqrt(2.), 0., 0.});
  EXPECT_NEAR(xyz.x_,   0., 1e-2);
  EXPECT_NEAR(xyz.y_, 150., 1e-2);
  EXPECT_NEAR(xyz.z_, 750., 1e-2);

  rot = l3->GetOrientation({100. * std::sqrt(2.), 0., 0.});
  EXPECT_NEAR(rot.roll_,         0., 1e-2);
  EXPECT_NEAR(rot.pitch_, -kPi / 4., 1e-2);
  EXPECT_NEAR(rot.yaw_,   -kPi / 2., 1e-2);
}




}  // namespace monolane
}  // namespace maliput

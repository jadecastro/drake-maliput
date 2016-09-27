#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

GTEST_TEST(MonolaneLanesTest, FlatLineLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz;
  api::Rotation rot;

  RoadGeometry rg = RoadGeometry({"apple"});
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l1 = s1->NewLineLane(
      {"l1"},
      {100., -75.}, {100., 50.},
      {-5., 5.}, {-10., 10.},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(l1->id().id_, "l1");
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);

  EXPECT_NEAR(l1->length(), std::sqrt((100. * 100) + (50. * 50.)), 1e-7);

  EXPECT_NEAR(l1->lane_bounds(0.).r_min_, -5., 1e-7);
  EXPECT_NEAR(l1->lane_bounds(0.).r_max_,  5., 1e-7);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_min_, -10., 1e-7);
  EXPECT_NEAR(l1->driveable_bounds(0.).r_max_,  10., 1e-7);

  xyz = l1->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100., 1e-2);
  EXPECT_NEAR(xyz.y_, -75., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  xyz = l1->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100 + (100. * (1. / l1->length())), 1e-2);
  EXPECT_NEAR(xyz.y_, -75 + ( 50. * (1. / l1->length())), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(xyz.x_, 100 + (-50. * (1. / l1->length())), 1e-2);
  EXPECT_NEAR(xyz.y_, -75 + (100. * (1. / l1->length())), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l1->ToGeoPosition({l1->length(), 0., 0.});
  EXPECT_NEAR(xyz.x_, 200., 1e-2);
  EXPECT_NEAR(xyz.y_, -25., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  // TODO(maddog) Test ToLanePosition().

  rot = l1->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw_, std::atan2(50., 100.), 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  rot = l1->GetOrientation({1., 0., 0.});
  EXPECT_NEAR(rot.yaw_, std::atan2(50., 100.), 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  rot = l1->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw_, std::atan2(50., 100.), 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  rot = l1->GetOrientation({l1->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw_, std::atan2(50., 100.), 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  // Derivative map should be identity (for a flat, straight road).
  api::LanePosition pdot;
  l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  l1->EvalMotionDerivatives({0., 0., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  l1->EvalMotionDerivatives({10., 5., 3.}, {1., 2., 3.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 2., 1e-6);
  EXPECT_NEAR(pdot.h_, 3., 1e-6);
}



GTEST_TEST(MonolaneLanesTest, FlatArcLane) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg = RoadGeometry({"apple"});
  Lane* l2 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {200., 200.}, 100., 0., M_PI / 2.,
      {-5., 5.}, {-10., 10.}, zp, zp);
  EXPECT_NEAR(l2->length(), 100. * M_PI / 2., 1e-7);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 300., 1e-2);
  EXPECT_NEAR(xyz.y_, 200., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  xyz = l2->ToGeoPosition({100. * M_PI / 2., 0., 0.});
  EXPECT_NEAR(xyz.x_, 200., 1e-2);
  EXPECT_NEAR(xyz.y_, 300., 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  rot = l2->GetOrientation({100. * M_PI / 2., 0., 0.});
  EXPECT_NEAR(rot.roll_,   0., 1e-2);
  EXPECT_NEAR(rot.pitch_,  0., 1e-2);
  EXPECT_NEAR(rot.yaw_,   M_PI, 1e-2);


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
  EXPECT_NEAR(rot.pitch_, -M_PI / 4., 1e-2);
  EXPECT_NEAR(rot.yaw_,   -M_PI / 2., 1e-2);
}




}  // namespace monolane
}  // namespace maliput

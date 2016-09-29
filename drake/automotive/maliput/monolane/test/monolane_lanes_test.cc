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
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

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
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      {100., -75.}, 100., 0.25 * M_PI, 1.5 * M_PI,
      {-5., 5.}, {-10., 10.},
      // Zero elevation, zero superelevation == flat.
      zp, zp);

  EXPECT_EQ(l2->id().id_, "l2");
  EXPECT_EQ(l2->segment(), s1);
  EXPECT_EQ(l2->index(), 0);
  EXPECT_EQ(l2->to_left(), nullptr);
  EXPECT_EQ(l2->to_right(), nullptr);

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, 1e-7);

  EXPECT_NEAR(l2->lane_bounds(0.).r_min_, -5., 1e-7);
  EXPECT_NEAR(l2->lane_bounds(0.).r_max_,  5., 1e-7);
  EXPECT_NEAR(l2->driveable_bounds(0.).r_min_, -10., 1e-7);
  EXPECT_NEAR(l2->driveable_bounds(0.).r_max_,  10., 1e-7);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos(0.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin(0.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  xyz = l2->ToGeoPosition({1., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos((0.25 * M_PI) + (1.5 / l2->length() * M_PI))), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin((0.25 * M_PI) + (1.5 / l2->length() * M_PI))), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l2->ToGeoPosition({0., 1., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos(0.25 * M_PI)) + (1. * std::cos(1.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin(0.25 * M_PI)) + (1. * std::sin(1.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  xyz = l2->ToGeoPosition({l2->length(), 0., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos(1.75 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin(1.75 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.z_, 0., 1e-2);

  // TODO(maddog) Test ToLanePosition().

  rot = l2->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw_, (0.25 + 0.5) * M_PI, 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  rot = l2->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw_, (0.25 + 0.5) * M_PI, 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  rot = l2->GetOrientation({l2->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw_, 0.25 * M_PI, 1e-6);  // 0.25 + 1.5 + 0.5
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0., 1e-6);

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, (100. / 90.) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, (100. / 110.) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);
  // ...and only r should matter for an otherwise flat arc.
  l2->EvalMotionDerivatives({l2->length(), -10., 100.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, (100. / 110.) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);
}



GTEST_TEST(MonolaneLanesTest, ArcLaneWithConstantSuperelevation) {
  CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  RoadGeometry rg = RoadGeometry({"apple"});
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  Lane* l2 = s1->NewArcLane(
      {"l2"},
      {100., -75.}, 100., 0.25 * M_PI, 1.5 * M_PI,
      {-5., 5.}, {-10., 10.},
      zp,
      { (0.10 * M_PI) / (100. * 1.5 * M_PI), 0., 0., 0. });

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, 1e-7);

  xyz = l2->ToGeoPosition({0., 0., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos(0.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin(0.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.z_,   0., 1e-2);

  xyz = l2->ToGeoPosition({0., 10., 0.});
  EXPECT_NEAR(xyz.x_, 100. + (100. * std::cos(0.25 * M_PI)) + (10. * std::cos(0.10 *M_PI) * std::cos(1.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.y_, -75. + (100. * std::sin(0.25 * M_PI)) + (10. * std::cos(0.10 * M_PI) * std::sin(1.25 * M_PI)), 1e-2);
  EXPECT_NEAR(xyz.z_, -10. * std::sin(0.10 * M_PI), 1e-2);


  // TODO(maddog) Test ToLanePosition().

  rot = l2->GetOrientation({0., 0., 0.});
  EXPECT_NEAR(rot.yaw_, (0.25 + 0.5) * M_PI, 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0.10 * M_PI, 1e-6);

  rot = l2->GetOrientation({0., 1., 0.});
  EXPECT_NEAR(rot.yaw_, (0.25 + 0.5) * M_PI, 1e-6);
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0.10 * M_PI, 1e-6);

  rot = l2->GetOrientation({l2->length(), 0., 0.});
  EXPECT_NEAR(rot.yaw_, 0.25 * M_PI, 1e-6);  // 0.25 + 1.5 + 0.5
  EXPECT_NEAR(rot.pitch_, 0., 1e-6);
  EXPECT_NEAR(rot.roll_, 0.10 * M_PI, 1e-6);

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 0., 1e-6);

  l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 0., 1e-6);
  EXPECT_NEAR(pdot.r_, 0., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, (100. / (100. - (10. * std::cos(0.10 * M_PI)))) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_, (100. / (100 + (10. * std::cos(0.10 * M_PI)))) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);

  // h matters, too.
  l2->EvalMotionDerivatives({l2->length(), -10., 8.}, {1., 1., 1.}, &pdot);
  EXPECT_NEAR(pdot.s_,
              (100. / (100
                       + (10. * std::cos(0.10 * M_PI))
                       - ( 8. * std::sin(0.10 * M_PI)))) * 1., 1e-6);
  EXPECT_NEAR(pdot.r_, 1., 1e-6);
  EXPECT_NEAR(pdot.h_, 1., 1e-6);
}



}  // namespace monolane
}  // namespace maliput

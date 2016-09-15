#include "utility/generate_obj.h"


#include "monolane/arc_lane.h"
#include "monolane/builder.h"
#include "monolane/junction.h"
#include "monolane/lane.h"
#include "monolane/line_lane.h"
#include "monolane/road_geometry.h"
#include "monolane/segment.h"

#include <iostream>

#include <gtest/gtest.h>


namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;
namespace mono = maliput::monolane;

TEST(GenerateObj, Podge) {
  mono::CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  const double kPi = 3.14159;
  const double kPi2 = kPi / 2.;

  mono::RoadGeometry rg = mono::RoadGeometry({"apple"});
  auto l0 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {100., 0.},
      {-5., 5.}, {-10., 10.},
      {0., 0., (60. / 100.), (-40. / 100)},
      zp);
  EXPECT_NEAR(20., l0->ToGeoPosition({l0->length(), 0., 0.}).z_, 1e-12);

  const double s50 = 50. * kPi2;
  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {100., 50.}, 50., -kPi2, kPi2,
      {-5., 5.}, {-10., 10.},
      {20. / s50, 0., 0., 0.},
      {0., 0., (0.6 / s50), (-0.4 / s50)});

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {100., 50.}, 50., 0., kPi2,
      {-5., 5.}, {-10., 10.},
      {20. / s50, 0., 0., 0.},
      {0.2 / s50, 0., (-0.6 / s50), (0.4 / s50)});

  rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {100., 100.}, {-100., 0.},
      {-5., 5.}, {-10., 10.},
      {(20. / 100), 0., (-60. / 100.), (40. / 100)},
      zp);

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {0., 50.}, 50., kPi2, kPi2,
      {-5., 5.}, {-10., 10.},
      zp,
      {0., 0., (1.5 / s50), (-1.0 / s50)});

  rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {0., 50.}, 50., kPi, kPi2,
      {-5., 5.}, {-10., 10.},
      zp,
      {0.5 / s50, 0., (-1.5 / s50), (1.0 / s50)});


  generate_obj(&rg, "/tmp/omg.obj", 1.);
}


TEST(GenerateObj, Fig8) {
  mono::CubicPolynomial zp {0., 0., 0., 0.};
  api::GeoPosition xyz {0., 0., 0.};
  api::Rotation rot {0., 0., 0.};

  const double kPi = 3.14159;
  const double kPi2 = kPi / 2.;

  mono::RoadGeometry rg = mono::RoadGeometry({"figure-eight"});
  api::RBounds lane_rb{-2., 2.};
  api::RBounds drive_rb{-4., 4.};
  auto l0 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {50., 0.},
      lane_rb, drive_rb,
      {0., 0., (3. * 3. / 50.), (-2 * 3. / 50.)},
      zp);
  EXPECT_NEAR(50., l0->length(), 0.1);
  EXPECT_NEAR(3., l0->ToGeoPosition({l0->length(), 0., 0.}).z_, 1e-12);

  // TODO(rick.poyner@tri.global): add curve superelevations.
  const double s50 = 50. * kPi2;
  auto l1 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {50., 50.}, 50., -kPi2, 1.5 * kPi2,
      lane_rb, drive_rb,
      {(3. / (s50 * 1.5)), 0., 0., 0.},
      zp);
  EXPECT_NEAR(3., l1->ToGeoPosition({0., 0., 0.}).z_, 1e-12);
  EXPECT_NEAR(3., l1->ToGeoPosition({l1->length(), 0., 0.}).z_, 1e-12);

  auto l2 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {50., 50.}, 50., 0.5 * kPi2, 1.5 * kPi2,
      lane_rb, drive_rb,
      {(3. / (s50 * 1.5)), 0., 0., 0.},
      zp);
  EXPECT_NEAR(3., l2->ToGeoPosition({0., 0., 0.}).z_, 1e-12);
  EXPECT_NEAR(3., l2->ToGeoPosition({l2->length(), 0., 0.}).z_, 1e-12);

  auto l3 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 50.}, {0., -50.},
      lane_rb, drive_rb,
      {(3. / 50.), 0., (3. * 3. / 50.), (-2 * 3. / 50.)},
      zp);
  EXPECT_NEAR(50., l3->length(), 0.1);
  EXPECT_NEAR(6., l3->ToGeoPosition({l3->length(), 0., 0.}).z_, 1e-12);

  auto l4 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {0., -50.},
      lane_rb, drive_rb,
      {(6. / 50.), 0., (3. * -3. / 50.), (-2 * -3. / 50.)},
      zp);
  EXPECT_NEAR(50., l4->length(), 0.1);
  EXPECT_NEAR(3., l4->ToGeoPosition({l4->length(), 0., 0.}).z_, 1e-12);

  // TODO(rick.poyner@tri.global): add curve superelevations.
  /*auto l5 =*/ rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l2"}, {-50., -50.}, 50., 0., -1.5 * kPi2,
      lane_rb, drive_rb,
      {(3. / (s50 * 1.5)), 0., 0., 0.},
      zp);
  //EXPECT_NEAR(50., l4->length(), 0.1);
  //EXPECT_NEAR(3., l5->ToGeoPosition({0., 0., 0.}).z_, 1e-12);
  //EXPECT_NEAR(3., l5->ToGeoPosition({l5->length(), 0., 0.}).z_, 1e-12);

  auto l6 = rg.NewJunction({"j2"})->NewSegment({"s2"})->NewArcLane(
      {"l6"}, {-50., -50.}, 50., -1.5 * kPi2, -1.5 * kPi2,
      lane_rb, drive_rb,
      {(3. / (s50 * 1.5)), 0., 0., 0.},
      zp);
  EXPECT_NEAR(3., l6->ToGeoPosition({0., 0., 0.}).z_, 1e-12);
  EXPECT_NEAR(3., l6->ToGeoPosition({l6->length(), 0., 0.}).z_, 1e-12);

  auto l7 = rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {-50., 0.}, {50., 0.},
      lane_rb, drive_rb,
      {(3. / 50.), 0., (3. * -3. / 50.), (-2 * -3. / 50.)},
      zp);
  EXPECT_NEAR(50., l7->length(), 0.1);
  EXPECT_NEAR(0., l7->ToGeoPosition({l7->length(), 0., 0.}).z_, 1e-12);

  generate_obj(&rg, "/tmp/wtf.obj", 1.);
}



TEST(GenerateObj, Hodge) {
  const double kPi = 3.14159;
  const double kPi2 = kPi / 2.;
  mono::Builder b({-5., 5.}, {-10., 10.});

  mono::XYZPoint start {{0., 0., 0.}, {0., 0., 0., 0.}};
  auto c1 = b.Connect("1", start, 100,
                      {20., 0., 0., 0.});
  auto c2 = b.Connect("2", c1->end(), mono::ArcOffset(50., kPi2),
                      {20., 0., 0.2, 0.});
  auto c3 = b.Connect("3", c2->end(), mono::ArcOffset(50., kPi2),
                      {20., 0., 0., 0.});
  auto c4 = b.Connect("4", c3->end(), 100,
                      {0., 0., 0., 0.});
  auto c5 = b.Connect("5", c4->end(), mono::ArcOffset(50., kPi2),
                      {0., 0., 0.5, 0.});
  /*auto c6 =*/ b.Connect("6", c5->end(), mono::ArcOffset(50., kPi2),
                      {0., 0., 0., 0.},
                      c1->start().xy_);
//SOON//  b.Connect("6", c5->end(), c1->begin());

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"apple"});
  generate_obj(rg.get(), "/tmp/omg2.obj", 1.);
}



} // namespace monolane
} // namespace maliput

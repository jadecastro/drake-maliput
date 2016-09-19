#include "drake/automotive/maliput/utility/generate_obj.h"


#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <iostream>

#include "gtest/gtest.h"


namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;
namespace mono = maliput::monolane;

GTEST_TEST(GenerateObj, Podge) {
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



GTEST_TEST(GenerateObj, Hodge) {
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
// SOON//  b.Connect("6", c5->end(), c1->begin());

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"apple"});
  generate_obj(rg.get(), "/tmp/omg2.obj", 1.);
}


GTEST_TEST(GenerateObj, Fig8Builder) {
  const double kPi = 3.14159;
  mono::Builder b({-2., 2.}, {-4., 4.});

  mono::XYZPoint start {{0., 0., -kPi / 4.}, {0., 0., 0., 0.}};
  auto c0 = b.Connect("0", start,
                      50., {3., 0., 0., 0.});

  auto c1 = b.Connect("1", c0->end(),
                      mono::ArcOffset(50., 0.75 * kPi), {3., 0., 0.4, 0.});
  auto c2 = b.Connect("2", c1->end(),
                      mono::ArcOffset(50., 0.75 * kPi), {3., 0., 0., 0.});

  auto c3 = b.Connect("3", c2->end(),
                      50., {6., 0., 0., 0.});
  auto c4 = b.Connect("4", c3->end(),
                      50., {3., 0., 0., 0.});

  auto c5 = b.Connect("5", c4->end(),
                      mono::ArcOffset(50., -0.75 * kPi), {3., 0., -0.4, 0.});
  auto c6 = b.Connect("6", c5->end(),
                      mono::ArcOffset(50., -0.75 * kPi), {3., 0., 0., 0.});

  /*auto c6 =*/ b.Connect("6", c6->end(),
                      50., {0., 0., 0., 0.});

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"figure-eight"});
  generate_obj(rg.get(), "/tmp/wtf2.obj", 1.);
}



GTEST_TEST(GenerateObj, DoubleRing) {
  const double kPi = 3.14159;
  mono::Builder b({-2., 2.}, {-4., 4.});

  mono::XYZPoint start {{0., 0., kPi / 2.}, {0., 0., 0., 0.}};
  auto cr0 = b.Connect("r0", start,
                      mono::ArcOffset(50., -0.25 * kPi), {0., 0., 0.0, 0.});
  auto cr1 = b.Connect("r1", cr0->end(),
                      mono::ArcOffset(50., -0.75 * kPi), {0., 0., -0.4, 0.});
  auto cr2 = b.Connect("r2", cr1->end(),
                      mono::ArcOffset(50., -0.75 * kPi), {0., 0., 0.0, 0.});
  auto cr3 = b.Connect("r3", cr2->end(),
                      mono::ArcOffset(50., -0.25 * kPi), {0., 0., 0.0, 0.});

  auto cl0 = b.Connect("l0", cr3->end(),
                      mono::ArcOffset(50., 0.25 * kPi), {0., 0., 0.0, 0.});
  auto cl1 = b.Connect("l1", cl0->end(),
                      mono::ArcOffset(50., 0.75 * kPi), {0., 0., 0.4, 0.});
  auto cl2 = b.Connect("l2", cl1->end(),
                      mono::ArcOffset(50., 0.75 * kPi), {0., 0., 0.0, 0.});
  /*auto cl3 =*/ b.Connect("l3", cl2->end(),
                      mono::ArcOffset(50., 0.25 * kPi), {0., 0., 0.0, 0.},
                      start.xy_);

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"double-ring"});
  generate_obj(rg.get(), "/tmp/double-ring.obj", 1.);
}



GTEST_TEST(GenerateObj, TeeIntersection) {
  const double kPi = 3.14159;
  mono::Builder b({-2., 2.}, {-4., 4.});

  mono::XYZPoint start {{0., 0., kPi / 2.}, {0., 0., 0., 0.}};
  auto cs = b.Connect("south",
                      {{0., -10., -kPi / 2.}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});
  auto cw = b.Connect("west",
                      {{-10., 0., kPi}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});
  auto ce = b.Connect("east",
                      {{10., 0., 0.}, {0., 0., 0., 0.}},
                      10., {0., 0., 0.0, 0.});

  /*auto csw =*/ b.Connect("south-west", cs->start().reverse(),
                       mono::ArcOffset(10., kPi / 2.), {0., 0., 0., 0.},
                       cw->start().xy_);
  /*auto cse =*/ b.Connect("south-east", cs->start().reverse(),
                       mono::ArcOffset(10., -kPi / 2.), {0., 0., 0., 0.},
                       ce->start().xy_);
  /*auto cew =*/ b.Connect("east-west",  ce->start().reverse(),
                       20., {0., 0., 0., 0.},
                       cw->start().xy_);

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"tee"});
  generate_obj(rg.get(), "/tmp/tee.obj", 1.);
}


GTEST_TEST(GenerateObj, Helix) {
  const double kPi = 3.14159;
  mono::Builder b({-2., 2.}, {-4., 4.});

  mono::XYZPoint start {{0., -10., 0.}, {0., 0., 0.4, 0.}};
  b.Connect("1", start,
            mono::ArcOffset(10., 4. * kPi), {20., 0., 0.4, 0.});

  std::unique_ptr<const api::RoadGeometry> rg = b.Build({"helix"});
  generate_obj(rg.get(), "/tmp/helix.obj", 1.);
}


}  // namespace monolane
}  // namespace maliput

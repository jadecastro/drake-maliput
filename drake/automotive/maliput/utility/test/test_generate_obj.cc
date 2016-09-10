#include "utility/generate_obj.h"


#include "monolane/arc_lane.h"
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
  rg.NewJunction({"j1"})->NewSegment({"s1"})->NewLineLane(
      {"l1"}, {0., 0.}, {100., 0.},
      {-5., 5.}, {-10., 10.},
      {0., 0., (60. / 100.), (-40. / 100)},
      zp);

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




} // namespace monolane
} // namespace maliput

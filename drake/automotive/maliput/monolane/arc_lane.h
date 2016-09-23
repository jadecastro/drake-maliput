#pragma once

#include <cmath>

#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class DRAKEAUTOMOTIVE_EXPORT ArcLane : public Lane {
 public:
  ArcLane(const api::LaneId& id, Segment* segment,
          const V2& center, const double radius,
          const double theta0, const double d_theta,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation)
      : Lane(id, segment,
             lane_bounds, driveable_bounds,
             radius * std::abs(d_theta),
             elevation, superelevation),
        r_(radius), cx_(center.x), cy_(center.y),
        theta0_(theta0), d_theta_(d_theta) {
    DRAKE_DEMAND(r_ > 0.);
  }

  api::LanePosition ToLanePosition(
      const api::GeoPosition& geo_pos) const override;

  void EvalMotionDerivatives(const api::LanePosition& position,
                             const api::IsoLaneVelocity& velocity,
                             const api::IsoLaneAcceleration& accel,
                             api::LanePosition* position_dot,
                             api::IsoLaneVelocity* velocity_dot) const override;

 private:
  V2 xy_of_p_(const double p) const override;
  double heading_of_p_(const double p) const override;

  double r_;
  double cx_;
  double cy_;
  double theta0_;
  double d_theta_;
};



}  // namespace monolane
}  // namespace maliput

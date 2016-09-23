#pragma once

#include "lane.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class DRAKEAUTOMOTIVE_EXPORT LineLane : public Lane {
 public:
  LineLane(const api::LaneId& id, Segment* segment,
           const V2& xy0, const V2& dxy,
           const api::RBounds& lane_bounds,
           const api::RBounds& driveable_bounds,
           const CubicPolynomial& elevation,
           const CubicPolynomial& superelevation)
      : Lane(id, segment,
             lane_bounds, driveable_bounds,
             dxy.length(),
             elevation, superelevation),
        x0_(xy0.x),
        y0_(xy0.y),
        dx_(dxy.x),
        dy_(dxy.y),
        heading_(std::atan2(dy_, dx_)) {}

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

  const double x0_;
  const double y0_;
  const double dx_;
  const double dy_;

  const double heading_;  // Memoized; derived from dy_, dx_.
};



}  // namespace monolane
}  // namespace maliput

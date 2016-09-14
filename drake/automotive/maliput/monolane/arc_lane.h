#pragma once

#include <cassert>
#include <cmath>

#include "lane.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class ArcLane : public Lane {
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
    assert(r_ > 0.);
  }

#if 0
  // TODO(maddog)  This belongs in a Builder.
  /// radius > 0: turn to left
  /// radius < 0: turn to right
  /// radius == 0:  forbidden
  ArcLane(const api::LaneId& id, Segment* segment,
          const V2& xy0, const double heading0, const V2& xy1,
          const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const CubicPolynomial& elevation,
          const CubicPolynomial& superelevation)
      : Lane(id, segment,
             lane_bounds, driveable_bounds, elevation, superelevation) {
    const V2 B = V2::midpoint(xy1, xy0);
    const V2 dP = V2::difference(xy1, xy0);
    double beta = dP.heading() - heading0;
    assert(beta != 0.);
    double g = 0.5 / std::tan(beta);
    double d = dP.length();

    const double PI = 3.14159; // TODO(maddog)  Get from somewhere else.
    r_ = 0.5 * d / std::sin(beta);
    cx_ = B.x - (g * dP.y);
    cy_ = B.y + (g * dP.x);
    if (r_ > 0.) {
      theta0_ = heading0 - (0.5 * PI);
    } else {
      theta0_ = heading0 + (0.5 * PI);
    }
    d_theta_ = 2. * beta;
  }
#endif

  virtual api::LanePosition ToLanePosition(
      const api::GeoPosition& geo_pos) const override;

  virtual void KinematicStep(double delta_t,
                             const api::LanePosition& current_lane_pos,
                             const api::LaneVelocity& current_lane_vel,
                             const api::LaneAcceleration& lane_accel,
                             const api::BranchChoices* choices,
                             const api::Lane** new_lane,
                             api::LanePosition* new_lane_pos,
                             api::LanePosition* new_lane_vel) const override;

 private:
  virtual V2 xy_of_p_(const double p) const override;
  virtual double heading_of_p_(const double p) const override;

  double r_;
  double cx_;
  double cy_;
  double theta0_;
  double d_theta_;

  double delta_r_min_;
  double delta_r_max_;
};



} // namespace monolane
} // namespace maliput

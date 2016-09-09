#include "arc_lane.h"

#include <cassert>
#include <cmath>

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


V2 ArcLane::xy_of_p_(const double p) const {
  const double theta = theta0_ + (p * d_theta_);
  return V2(cx_ + (r_ * std::cos(theta)),
            cy_ + (r_ * std::sin(theta)));
}


double ArcLane::heading_of_p_(const double p) const {
  const double kPi = 3.14159;
  const double theta = theta0_ + (p * d_theta_);
  return theta + std::copysign(kPi / 2.0, d_theta_);
}


api::LanePosition ArcLane::ToLanePosition(
    const api::GeoPosition& geo_pos) const {
  (geo_pos.x_); // TODO maddog temp warning quashing
  assert(0); // TODO maddog Implement me.
  }



void ArcLane::KinematicStep(double delta_t,
                             const api::LanePosition& current_lane_pos,
                             const api::LaneVelocity& current_lane_vel,
                             const api::LaneAcceleration& lane_accel,
                             const api::BranchChoices* choices,
                             const api::Lane** new_lane,
                             api::LanePosition* new_lane_pos,
                             api::LanePosition* new_lane_vel) const {
  (&delta_t);
  (&current_lane_pos);
  (&current_lane_vel);
  (&lane_accel);
  (&choices);
  (&new_lane);
  (&new_lane_pos);
  (&new_lane_vel); // TODO maddog temp warning quashing
  assert(0); // TODO maddog Implement me.
}

} // namespace monolane
} // namespace maliput

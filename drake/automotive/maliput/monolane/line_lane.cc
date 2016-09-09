#include "line_lane.h"

#include <cassert>
#include <cmath>

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


V2 LineLane::xy_of_p_(const double p) const {
  return V2(x0_ + (p * dx_),
            y0_ + (p * dy_));
}


// TODO(maddog)  Memoize this constant.
double LineLane::heading_of_p_(const double) const {
  return std::atan2(dy_, dx_);
}


api::LanePosition LineLane::ToLanePosition(
    const api::GeoPosition& geo_pos) const {
  (geo_pos.x_); // TODO maddog temp warning quashing
  assert(0); // TODO maddog Implement me.
  }


void LineLane::KinematicStep(double delta_t,
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

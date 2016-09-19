#include "drake/automotive/maliput/monolane/line_lane.h"

#include <cmath>

#include "drake/automotive/maliput/monolane/ignore.h"
#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


V2 LineLane::xy_of_p_(const double p) const {
  return V2(x0_ + (p * dx_),
            y0_ + (p * dy_));
}


double LineLane::heading_of_p_(const double) const { return heading_; }


api::LanePosition LineLane::ToLanePosition(
    const api::GeoPosition& geo_pos) const {
  ignore(geo_pos.x_);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
  return {0, 0, 0};
}


void LineLane::KinematicStep(double delta_t,
                             const api::LanePosition& current_lane_pos,
                             const api::LaneVelocity& current_lane_vel,
                             const api::LaneAcceleration& lane_accel,
                             const api::BranchChoices* choices,
                             const api::Lane** new_lane,
                             api::LanePosition* new_lane_pos,
                             api::LanePosition* new_lane_vel) const {
  ignore(&delta_t);
  ignore(&current_lane_pos);
  ignore(&current_lane_vel);
  ignore(&lane_accel);
  ignore(&choices);
  ignore(&new_lane);
  ignore(&new_lane_pos);
  ignore(&new_lane_vel);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}

}  // namespace monolane
}  // namespace maliput

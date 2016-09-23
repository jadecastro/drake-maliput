#include "drake/automotive/maliput/monolane/arc_lane.h"

#include "drake/automotive/maliput/monolane/ignore.h"

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
  const double theta = theta0_ + (p * d_theta_);
  return theta + std::copysign(M_PI / 2.0, d_theta_);
}


api::LanePosition ArcLane::ToLanePosition(
    const api::GeoPosition& geo_pos) const {
  ignore(geo_pos.x_);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}



void ArcLane::EvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity,
    const api::IsoLaneAcceleration& accel,
    api::LanePosition* position_dot,
    api::IsoLaneVelocity* velocity_dot) const {
  ignore(&position);
  ignore(&velocity);
  ignore(&accel);
  ignore(&position_dot);
  ignore(&velocity_dot);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}

}  // namespace monolane
}  // namespace maliput

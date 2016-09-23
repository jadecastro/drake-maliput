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


V2 LineLane::xy_dot_of_p_(const double p) const {
  return V2(dx_,
            dy_);
}


double LineLane::heading_of_p_(const double) const { return heading_; }


double LineLane::heading_dot_of_p_(const double) const { return 0.; }


api::LanePosition LineLane::ToLanePosition(
    const api::GeoPosition& geo_pos) const {
  ignore(geo_pos.x_);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}


}  // namespace monolane
}  // namespace maliput

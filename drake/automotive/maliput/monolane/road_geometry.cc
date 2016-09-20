#include "drake/automotive/maliput/monolane/road_geometry.h"

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/ignore.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(std::make_unique<Junction>(id, this));
  return junctions_.back().get();
}


BranchPoint* RoadGeometry::NewBranchPoint(api::BranchPointId id) {
  branch_points_.push_back(std::make_unique<BranchPoint>(id, this));
  return branch_points_.back().get();
}


const api::Junction* RoadGeometry::junction(int index) const {
  return junctions_[index].get();
}


const api::BranchPoint* RoadGeometry::branch_point(int index) const {
  return branch_points_[index].get();
}




api::RoadPosition RoadGeometry::ToRoadPosition(
    const api::GeoPosition& geo_pos,
    const api::RoadPosition& hint) const {
  ignore(&geo_pos);
  ignore(&hint);  // TODO(maddog) temp warning quashing
  DRAKE_ABORT();  // TODO(maddog) Implement me.
}



}  // namespace monolane
}  // namespace maliput

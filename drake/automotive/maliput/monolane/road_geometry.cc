#include "road_geometry.h"

#include <cassert>

#include "branch_point.h"
#include "junction.h"
#include "make_unique.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

Junction* RoadGeometry::NewJunction(api::JunctionId id) {
  junctions_.push_back(make_unique<Junction>(id, this));
  return junctions_.back().get();
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
  (&geo_pos);
  (&hint); // TODO maddog temp warning quashing
  assert(0); // TODO maddog Implement me.
}



} // namespace monolane
} // namespace maliput

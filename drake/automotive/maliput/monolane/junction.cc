#include "junction.h"

#include "make_unique.h"
#include "road_geometry.h"
#include "segment.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;



const api::RoadGeometry* Junction::road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id) {
  segments_.push_back(make_unique<Segment>(id, this));
  return segments_.back().get();;
}


const api::Segment* Junction::segment (int index) const {
  return segments_[index].get();
}

} // namespace monolane
} // namespace maliput

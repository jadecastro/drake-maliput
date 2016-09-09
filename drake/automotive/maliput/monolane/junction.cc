#include "junction.h"

#include "road_geometry.h"
#include "segment.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;



const api::RoadGeometry* Junction::road_geometry() const {
  return road_geometry_;
}


Segment* Junction::NewSegment(api::SegmentId id) {
  Segment* segment = new Segment(id, this);
  segments_.push_back(segment);
  return segment;
}


const api::Segment* Junction::segment (int index) const {
  return segments_[index];
}

} // namespace monolane
} // namespace maliput

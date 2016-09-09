#pragma once

#include <vector>

#include "geometry_api/junction.h"
#include "geometry_api/road_geometry.h"
#include "geometry_api/segment.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class RoadGeometry;
class Segment;

class Junction : public api::Junction {
 public:
  Junction(const api::JunctionId& id, RoadGeometry* rg)
      : id_(id), road_geometry_(rg) {}

  Segment* NewSegment(api::SegmentId id);

  virtual const api::JunctionId id() const {
    return id_;
  }

  virtual const api::RoadGeometry* road_geometry() const;

  virtual int num_segments() const {
    return segments_.size();
  }

  virtual const api::Segment* segment (int index) const;

 private:
  api::JunctionId id_;
  RoadGeometry* road_geometry_;
  std::vector<Segment*> segments_;
};


} // namespace monolane
} // namespace maliput

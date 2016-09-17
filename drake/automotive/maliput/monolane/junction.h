#pragma once

#include <memory>
#include <vector>

#include "segment.h"

#include "drake/automotive/maliput/geometry_api/junction.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"
#include "drake/automotive/maliput/geometry_api/segment.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class RoadGeometry;

class DRAKEAUTOMOTIVE_EXPORT Junction : public api::Junction {
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
  std::vector<std::unique_ptr<Segment>> segments_;
};


} // namespace monolane
} // namespace maliput

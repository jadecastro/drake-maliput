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

 private:
  const api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override;

  api::JunctionId id_;
  RoadGeometry* road_geometry_;
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace monolane
}  // namespace maliput

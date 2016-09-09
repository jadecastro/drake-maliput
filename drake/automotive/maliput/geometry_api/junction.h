#pragma once

#include "state.h"

namespace maliput {
namespace geometry_api {

class RoadGeometry;
class Segment;

class Junction {
 public:
  virtual ~Junction() {}

  // Provide persistent ID.
  virtual const JunctionId id() const = 0;

  virtual const RoadGeometry* road_geometry() const = 0;

  virtual int num_segments() const = 0;

  virtual const Segment* segment(int index) const = 0;
};


} // namespace geometry_api
} // namespace maliput

#pragma once

#include "state.h"

namespace maliput {
namespace geometry_api {

class Junction;
class Lane;

class DRAKEAUTOMOTIVE_EXPORT Segment {
 public:
  virtual ~Segment() {}

  // Provide persistent ID.
  virtual const SegmentId id() const = 0;

  virtual const Junction* junction() const = 0;

  virtual int num_lanes() const = 0;
  // index order is meaningful == adjacency.
  virtual const Lane* lane(int index) const = 0;
};



} // namespace geometry_api
} // namespace maliput

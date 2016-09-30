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
  const SegmentId id() const { return do_id(); }

  const Junction* junction() const { return do_junction(); }

  int num_lanes() const { return do_num_lanes(); }

  // index order is meaningful == adjacency.
  const Lane* lane(int index) const { return do_lane(index); }

 private:
  // Provide persistent ID.
  virtual const SegmentId do_id() const = 0;

  virtual const Junction* do_junction() const = 0;

  virtual int do_num_lanes() const = 0;

  // index order is meaningful == adjacency.
  virtual const Lane* do_lane(int index) const = 0;
};



}  // namespace geometry_api
}  // namespace maliput

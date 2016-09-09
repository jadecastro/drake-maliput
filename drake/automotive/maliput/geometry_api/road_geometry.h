#pragma once

#include "state.h"

namespace maliput {
namespace geometry_api {

class BranchPoint;
class Junction;

class RoadGeometry {
 public:
  virtual ~RoadGeometry() {}

  // Provide persistent ID.
  virtual const RoadGeometryId id() const = 0;

  virtual int num_junctions() const = 0;

  virtual const Junction* junction(int index) const = 0;

  virtual int num_branch_points() const = 0;

  virtual const BranchPoint* branch_point(int index) const = 0;


  virtual RoadPosition ToRoadPosition(const GeoPosition& geo_pos,
                                      const RoadPosition& hint) const = 0;
};


} // namespace geometry_api
} // namespace maliput

#pragma once

#include "state.h"

namespace maliput {
namespace geometry_api {

class BranchPoint;
class Junction;

/// Abstract API for the geometry of a road network, including both
/// the network topology and the geometry of its embedding in 3-space.
class DRAKEAUTOMOTIVE_EXPORT RoadGeometry {
 public:
  virtual ~RoadGeometry() {}

  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  virtual const RoadGeometryId id() const = 0;

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Junctions in the RoadGeometry.
  virtual int num_junctions() const = 0;

  /// @return the Junction indexed by @param index.
  virtual const Junction* junction(int index) const = 0;

  /// @return the number of BrancPoints n the RoadGeometry.
  virtual int num_branch_points() const = 0;

  /// @return the BranchPoint indexed by @param index.
  virtual const BranchPoint* branch_point(int index) const = 0;


  /// Figure out the RoadPosition corresponding to GeoPosition @param geo_pos,
  /// potentially using @param hint to guide the search.
  // TODO(maddog)  UNDER CONSTRUCTION
  virtual RoadPosition ToRoadPosition(const GeoPosition& geo_pos,
                                      const RoadPosition& hint) const = 0;
};


} // namespace geometry_api
} // namespace maliput

#pragma once

#include "state.h"

namespace maliput {
namespace geometry_api {

class RoadGeometry;
class Segment;

/// A Junction is a closed set of Segments which physically overlap,
/// in the sense that RoadPositions in the domains of two Segments map
/// to the same GeoPosition.  The Segments need not be directly
/// connected to one another in the network topology.
///
/// Junctions are owned/managed by RoadGeometry.
class DRAKEAUTOMOTIVE_EXPORT Junction {
 public:
  virtual ~Junction() {}

  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  virtual const JunctionId id() const = 0;

  /// @return the RoadGeometry which owns this Junction.
  virtual const RoadGeometry* road_geometry() const = 0;

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Segments in the Junction.
  virtual int num_segments() const = 0;

  /// @return the Segment indexed by @param index.
  virtual const Segment* segment(int index) const = 0;
};


}  // namespace geometry_api
}  // namespace maliput

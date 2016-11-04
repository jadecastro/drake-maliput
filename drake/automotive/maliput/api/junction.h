#pragma once

namespace drake {
namespace maliput {
namespace api {

class RoadGeometry;
class Segment;


struct DRAKE_EXPORT JunctionId {
  std::string id;
};


/// A Junction is a closed set of Segments which physically overlap,
/// in the sense that RoadPositions in the domains of two Segments map
/// to the same GeoPosition.  The Segments need not be directly
/// connected to one another in the network topology.
///
/// Junctions are owned/managed by RoadGeometry.
class DRAKE_EXPORT Junction {
 public:
  virtual ~Junction() {}

  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  const JunctionId id() const { return do_id(); }

  /// @return the RoadGeometry which owns this Junction.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Segments in the Junction.
  int num_segments() const { return do_num_segments(); }

  /// @return the Segment indexed by @param index.
  const Segment* segment(int index) const { return do_segment(index); }

 private:
  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  virtual const JunctionId do_id() const = 0;

  /// @return the RoadGeometry which owns this Junction.
  virtual const RoadGeometry* do_road_geometry() const = 0;

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Segments in the Junction.
  virtual int do_num_segments() const = 0;

  /// @return the Segment indexed by @param index.
  virtual const Segment* do_segment(int index) const = 0;
};


}  // namespace geometry_api
}  // namespace maliput
}  // namespace drake

#pragma once

namespace drake {
namespace maliput {
namespace api {

class Junction;
class Lane;


/// Persistent identifier for a Segment element.
struct DRAKE_EXPORT SegmentId {
  std::string id;
};


/// A Segment represents a bundle of adjacent Lanes which share a
/// continuously traversable road surface.  Every LanePosition on a
/// given Lane of a Segment has a corresponding LanePosition on each
/// other Lane, all with the same height-above-surface h, that all
/// map to the same GeoPoint in 3-space.
///
/// Segments are grouped by Junctions.
class DRAKE_EXPORT Segment {
 public:
  virtual ~Segment() {}

  /// @return the persistent identifier.
  const SegmentId id() const { return do_id(); }

  /// @return the Junction to which this Segment belongs.
  const Junction* junction() const { return do_junction(); }

  /// @return the number of Lanes contained in this Segment.
  ///
  /// Return value is non-negative.
  int num_lanes() const { return do_num_lanes(); }

  /// @return the Lane indexed by @p index.
  /// The indexing order is meaningful; numerically adjacent indices correspond
  /// to geometrically adjacent Lanes.
  ///
  /// @pre @p index must be >= 0 and < num_lanes().
  const Lane* lane(int index) const { return do_lane(index); }

 private:
  /// NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the public methods.
  //@{
  virtual const SegmentId do_id() const = 0;

  virtual const Junction* do_junction() const = 0;

  virtual int do_num_lanes() const = 0;

  virtual const Lane* do_lane(int index) const = 0;
  //@}
};



}  // namespace api
}  // namespace maliput
}  // namespace drake

#pragma once

#include "drake/automotive/maliput/api/lane_data.h"

#include <memory>

namespace drake {
namespace maliput {
namespace api {

class RoadGeometry;

// Persistent identifier for a BranchPoint element.
struct DRAKE_EXPORT BranchPointId {
  std::string id;
};


/// A set of LaneEnds.
class DRAKE_EXPORT LaneEndSet {
 public:
  virtual ~LaneEndSet() {}

  /// @return the number of LaneEnds in this set.
  ///
  /// Return value is non-negative.
  int size() const { return do_size(); }

  /// @return the LaneEnd indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < size().
  const LaneEnd& get(int index) const { return do_get(index); }

 private:
  /// NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the public methods.
  //@{
  virtual int do_size() const = 0;

  virtual const LaneEnd& do_get(int index) const = 0;
  //@}
};


/// A BranchPoint is a node in the network of a RoadGeometry at which
/// Lanes connect to one another.  A BranchPoint is a collection of LaneEnds
/// specifying the Lanes (and, in particular, which ends of the Lanes) are
/// connected at the BranchPoint.
///
/// LaneEnds participating in a BranchPoint are grouped into two sets,
/// arbitrarily named "A-side" and "B-side".  LaneEnds on the same "side"
/// have coincident into-the-lane tangent vectors, which are anti-parallel
/// to those of LaneEnds on the other side.
class DRAKE_EXPORT BranchPoint {
 public:
  virtual ~BranchPoint() {}

  /// @return the persistent identifier.
  const BranchPointId id() const { return do_id(); }

  /// @return the RoadGeometry to which this BranchPoint belongs.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  /// @return the set of LaneEnds on the same side as the given @p end,
  /// e.g., the LaneEnds merging with the given @p end.
  ///
  /// The returned set includes the given @p end.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  const LaneEndSet* GetConfluentBranches(const LaneEnd& end) const {
    return DoGetConfluentBranches(end);
  }

  /// @return the set of LaneEnds on the other side from the given @p end,
  /// e.g., the LaneEnds which @p end flows into.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  const LaneEndSet* GetOngoingBranches(const LaneEnd& end) const {
    return DoGetOngoingBranches(end);
  }

  /// @return the default ongoing branch (if any) for the given @p end.
  /// This typically represents what would be considered "continuing through
  /// traffic" from @p end (e.g., as opposed to a branch executing a turn).
  ///
  /// If @p end has no default-branch at this BranchPoint, the return
  /// value will be nullptr.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  // TODO(maddog@tri.global)  The return type yearns to be
  //                          const boost::optional<LaneEnd>&.
  std::unique_ptr<LaneEnd> GetDefaultBranch(const LaneEnd& end) const {
    return std::move(DoGetDefaultBranch(end));
  }

  /// @return the set of LaneEnds grouped together on the "A-side".
  const LaneEndSet* GetASide() const { return DoGetASide(); }

  /// @return the set of LaneEnds grouped together on the "B-side".
  const LaneEndSet* GetBSide() const { return DoGetBSide(); }

 private:
  /// NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the public methods.
  //@{
  virtual const BranchPointId do_id() const = 0;

  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual const LaneEndSet* DoGetConfluentBranches(
      const LaneEnd& end) const = 0;

  virtual const LaneEndSet* DoGetOngoingBranches(
      const LaneEnd& end) const = 0;

  virtual std::unique_ptr<LaneEnd> DoGetDefaultBranch(
      const LaneEnd& end) const = 0;

  virtual const LaneEndSet* DoGetASide() const = 0;

  virtual const LaneEndSet* DoGetBSide() const = 0;
  //@}
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
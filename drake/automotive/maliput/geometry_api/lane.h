#pragma once

#include "state.h"

#include <boost/optional.hpp>

namespace maliput {
namespace geometry_api {


class BranchPoint;
class Segment;
class SetOfLaneEnds;

class BranchChoices;  // TODO(maddog) define me.


class DRAKE_EXPORT Lane {
 public:
  virtual ~Lane() {}

  /// Returns a persistent identifier for this Lane.
  const LaneId id() const { return do_id(); }

  /// Returns a non-null pointer to the Segment to which this Lane belongs.
  const Segment* segment() const { return do_segment(); }

  /// Returns the index of this Lane within the Segment which owns it.
  int index() const { return do_index(); }

  /// Returns a pointer to the adjacent Lane to the left of this Lane.
  ///
  /// Left is considered the +r direction with regards to the (s,r,h) frame,
  /// e.g., "to the left along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the left.
  const Lane* to_left() const { return do_to_left(); }

  /// Returns a pointer to the adjacent Lane to the right of this Lane.
  ///
  /// Right is considered the -r direction with regards to the (s,r,h) frame,
  /// e.g., "to the right along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the right.
  const Lane* to_right() const { return do_to_right(); }

  /// Returns the arc-length of the lane along its reference curve.
  ///
  /// The value of length() is also the maximum s-coordinate for this lane;
  /// i.e., the domain of s is [0, length()].
  double length() const { return do_length(); }

  /// Returns the nominal lateral (r) bounds for the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "staying in the lane".
  RBounds lane_bounds(double s) const { return do_lane_bounds(s); }

  /// Returns the driveable lateral (r) bounds of the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "on pavement", reflecting the physical extent of the paved surface of
  /// the lane's segment.
  RBounds driveable_bounds(double s) const { return do_driveable_bounds(s); }

  /// Returns the GeoPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::driveable_bounds().
  GeoPosition ToGeoPosition(const LanePosition& lane_pos) const {
    return DoToGeoPosition(lane_pos);
  }

  LanePosition ToLanePosition(const GeoPosition& geo_pos) const {
    return DoToLanePosition(geo_pos);
  }

  // TODO(maddog)  Method to convert LanePosition to that of another Lane.
  //               (Should assert that both lanes belong to same Segment.)

  /// Return the rotation which transforms LANE basis into GEO basis.
  Rotation GetOrientation(const LanePosition& lane_pos) const {
    return DoGetOrientation(lane_pos);
  }

  void EvalMotionDerivatives(const LanePosition& position,
                             const IsoLaneVelocity& velocity,
                             LanePosition* position_dot) const {
    DoEvalMotionDerivatives(position, velocity, position_dot);
  }

  // TODO(maddog) void EvalSurfaceDerivatives(...) const { return do_(); }


  /// Returns the lane's BranchPoint for the end specificed by @p which_end.
  const BranchPoint* GetBranchPoint(const LaneEnd::Which which_end) const {
    return DoGetBranchPoint(which_end);
  }

  /// Returns the set of ongoing lanes connected to the lane at @p which_end.
  const SetOfLaneEnds* GetBranches(const LaneEnd::Which which_end) const {
    return DoGetBranches(which_end);
  }

  /// Returns the default ongoing lane connected at @p which_end.
  ///
  /// @returns boost::none if no default branch has been established
  ///          at @p which_end.
  const boost::optional<LaneEnd>& GetDefaultBranch(
      const LaneEnd::Which which_end) const {
    return DoGetDefaultBranch(which_end);
  }

 private:
    /// Returns a persistent identifier for this Lane.
  virtual const LaneId do_id() const = 0;

  /// Returns a non-null pointer to the Segment to which this Lane belongs.
  virtual const Segment* do_segment() const = 0;

  /// Returns the index of this Lane within the Segment which owns it.
  virtual int do_index() const = 0;

  /// Returns a pointer to the adjacent Lane to the left of this Lane.
  ///
  /// Left is considered the +r direction with regards to the (s,r,h) frame,
  /// e.g., "to the left along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the left.
  virtual const Lane* do_to_left() const = 0;

  /// Returns a pointer to the adjacent Lane to the right of this Lane.
  ///
  /// Right is considered the -r direction with regards to the (s,r,h) frame,
  /// e.g., "to the right along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the right.
  virtual const Lane* do_to_right() const = 0;

  /// Returns the arc-length of the lane along its reference curve.
  ///
  /// The value of length() is also the maximum s-coordinate for this lane;
  /// i.e., the domain of s is [0, length()].
  virtual double do_length() const = 0;

  /// Returns the nominal lateral (r) bounds for the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "staying in the lane".
  virtual RBounds do_lane_bounds(double s) const = 0;

  /// Returns the driveable lateral (r) bounds of the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "on pavement", reflecting the physical extent of the paved surface of
  /// the lane's segment.
  virtual RBounds do_driveable_bounds(double s) const = 0;

  /// Returns the GeoPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::driveable_bounds().
  virtual GeoPosition DoToGeoPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoToLanePosition(const GeoPosition& geo_pos) const = 0;

  // TODO(maddog)  Method to convert LanePosition to that of another Lane.
  //               (Should assert that both lanes belong to same Segment.)

  /// Return the rotation which transforms LANE basis into GEO basis.
  virtual Rotation DoGetOrientation(const LanePosition& lane_pos) const = 0;


  virtual void DoEvalMotionDerivatives(const LanePosition& position,
                                       const IsoLaneVelocity& velocity,
                                       LanePosition* position_dot) const = 0;

  // TODO(maddog) virtual void EvalSurfaceDerivatives(...) const = 0;


  /// Returns the lane's BranchPoint for the end specificed by @p which_end.
  virtual const BranchPoint* DoGetBranchPoint(
      const LaneEnd::Which which_end) const = 0;

  /// Returns the set of ongoing lanes connected to the lane at @p which_end.
  virtual const SetOfLaneEnds* DoGetBranches(
      const LaneEnd::Which which_end) const = 0;

  /// Returns the default ongoing lane connected at @p which_end.
  ///
  /// @returns boost::none if no default branch has been established
  ///          at @p which_end.
  virtual const boost::optional<LaneEnd>& DoGetDefaultBranch(
      const LaneEnd::Which which_end) const = 0;
};

}  // namespace geometry_api
}  // namespace maliput

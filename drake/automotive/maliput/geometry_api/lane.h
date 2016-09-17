#pragma once

#include "state.h"

#include <boost/optional.hpp>

namespace maliput {
namespace geometry_api {


class BranchPoint;
class Segment;
class SetOfLaneEnds;

class BranchChoices; // TODO maddog define me.


class DRAKEAUTOMOTIVE_EXPORT Lane {
 public:

  enum class Endpoint {
    kStart,
      kEnd,
      };

  virtual ~Lane() {}

  /// @return a persistent identifier.
  virtual const LaneId id() const = 0;


  virtual const Segment* segment() const = 0;

  virtual int index() const = 0;


  virtual const Lane* to_left() const = 0;

  virtual const Lane* to_right() const = 0;

  // TODO maddog Accessors for rmin, rmax as function of s.

  /// Return the arc-length of the lane.
  ///
  /// Domain of the s-component for this lane is [0, length()].
  virtual double length() const = 0;

  virtual RBounds lane_bounds(double s) const = 0;

  virtual RBounds driveable_bounds(double s) const = 0;

  virtual GeoPosition ToGeoPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePosition ToLanePosition(const GeoPosition& geo_pos) const = 0;

  // TODO(maddog)  Method to convert LanePosition to that of another Lane.
  //               (Should assert that both lanes belong to same Segment.)

  /// Return the rotation which transforms LANE basis into GEO basis.
  virtual Rotation GetOrientation(const LanePosition& lane_pos) const = 0;


  virtual void KinematicStep(double delta_t,
                             const LanePosition& current_lane_pos,
                             const LaneVelocity& current_lane_vel,
                             const LaneAcceleration& lane_accel,
                             const BranchChoices* choices,
                             const Lane** new_lane,
                             LanePosition* new_lane_pos,
                             LanePosition* new_lane_vel) const = 0;

  virtual const BranchPoint* GetBranchPoint(
      const LaneEnd::Which which_end) const = 0;

  virtual const SetOfLaneEnds* GetBranches(
      const LaneEnd::Which which_end) const = 0;

  virtual const boost::optional<LaneEnd>& GetDefaultBranch(
      const LaneEnd::Which which_end) const = 0;
};






} // namespace geometry_api
} // namespace maliput

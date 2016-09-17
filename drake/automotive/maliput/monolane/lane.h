#pragma once

#include "drake/automotive/maliput/geometry_api/branch_point.h"
#include "drake/automotive/maliput/geometry_api/lane.h"
#include "drake/automotive/maliput/geometry_api/segment.h"

#include "mathiness.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class BranchPoint;
class Segment;


class DRAKEAUTOMOTIVE_EXPORT Lane : public api::Lane {
 public:
  Lane(const api::LaneId& id, Segment* segment,
       const api::RBounds& lane_bounds,
       const api::RBounds& driveable_bounds,
       const double p_scale,
       const CubicPolynomial& elevation,
       const CubicPolynomial& superelevation)
      : id_(id), segment_(segment),
        lane_bounds_(lane_bounds),
        driveable_bounds_(driveable_bounds),
        p_scale_(p_scale),
        elevation_(elevation),
        superelevation_(superelevation) {}

  // TODO(maddog) Explain clearly how/why the elevation and superelevation
  //              functions need to be isotropically scaled by the xy-projected
  //              arc-length of the xy-primitive curve.


  // TODO(maddog) Allow lane centerline to be offset from "segment ref line",
  //              so that superelevation can have a center-of-rotation which
  //              is different from r=0.

  virtual ~Lane() {}

  virtual const api::LaneId id() const override { return id_; }

  virtual const api::Segment* segment() const override;

  virtual int index() const override { return 0; }  // Only one lane per segment!

  virtual const api::Lane* to_left() const override { return nullptr; }

  virtual const api::Lane* to_right() const override { return nullptr; }

  virtual const api::BranchPoint* GetBranchPoint(
      const api::LaneEnd::Which which_end) const override;

  virtual const api::SetOfLaneEnds* GetBranches(
      const api::LaneEnd::Which which_end) const override;

  virtual const boost::optional<api::LaneEnd>& GetDefaultBranch(
      const api::LaneEnd::Which which_end) const override;

  virtual double length() const override;

  virtual api::RBounds lane_bounds(double) const override {
    return lane_bounds_;
  }

  virtual api::RBounds driveable_bounds(double) const override {
    return driveable_bounds_;
  }

  virtual api::GeoPosition ToGeoPosition(
      const api::LanePosition& lane_pos) const override;

  virtual api::Rotation GetOrientation(
      const api::LanePosition& lane_pos) const override;


  const CubicPolynomial& elevation() const { return elevation_; }
  const CubicPolynomial& superelevation() const { return superelevation_; }

 private:
  virtual V2 xy_of_p_(const double p) const = 0;
  virtual double heading_of_p_(const double p) const = 0;

  Rot3 rot3_of_p_(const double p) const;

  const api::LaneId id_;
  const Segment* segment_;
  BranchPoint* start_bp_;
  BranchPoint* end_bp_;

  const api::RBounds lane_bounds_;
  const api::RBounds driveable_bounds_;
  const double p_scale_;
  // Common elevation and superelevation structures.
  const CubicPolynomial elevation_;
  const CubicPolynomial superelevation_;

};


} // namespace monolane
} // namespace maliput

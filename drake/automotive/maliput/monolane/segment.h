#pragma once

#include <cassert>

#include "geometry_api/junction.h"
#include "geometry_api/lane.h"
#include "geometry_api/segment.h"

#include "mathiness.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

class Junction;
class Lane;
class ArcLane;
class LineLane;

class Segment : public api::Segment {
 public:
  Segment(const api::SegmentId& id, Junction* j)
      : id_(id), junction_(j) {}

  LineLane* NewLineLane(api::LaneId id,
                        const V2& xy0, const V2& dxy,
                        const api::RBounds& lane_bounds,
                        const api::RBounds& driveable_bounds,
                        const CubicPolynomial& elevation,
                        const CubicPolynomial& superelevation);

  ArcLane* NewArcLane(api::LaneId id,
                      const V2& center, const double radius,
                      const double theta0, const double d_theta,
                      const api::RBounds& lane_bounds,
                      const api::RBounds& driveable_bounds,
                      const CubicPolynomial& elevation,
                      const CubicPolynomial& superelevation);

  virtual const api::SegmentId id() const { return id_; }

  virtual const api::Junction* junction() const;

  virtual int num_lanes() const { return 1; }

  virtual const api::Lane* lane(int index) const;

 private:
  api::SegmentId id_;
  Junction* junction_;
  Lane* lane_ {};
};



} // namespace monolane
} // namespace maliput

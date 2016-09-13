#include "segment.h"

#include "arc_lane.h"
#include "junction.h"
#include "lane.h"
#include "line_lane.h"
#include "make_unique.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

const api::Junction* Segment::junction() const {
  return junction_;
}


LineLane* Segment::NewLineLane(api::LaneId id,
                               const V2& xy0, const V2& dxy,
                               const api::RBounds& lane_bounds,
                               const api::RBounds& driveable_bounds,
                               const CubicPolynomial& elevation,
                               const CubicPolynomial& superelevation) {
  assert(lane_.get() == nullptr);
  std::unique_ptr<LineLane> lane = make_unique<LineLane>(
      id, this, xy0, dxy,
      lane_bounds, driveable_bounds,
      elevation, superelevation);
  LineLane* result = lane.get();
  lane_ = std::move(lane);
  return result;
}


ArcLane* Segment::NewArcLane(api::LaneId id,
                             const V2& center, const double radius,
                             const double theta0, const double d_theta,
                             const api::RBounds& lane_bounds,
                             const api::RBounds& driveable_bounds,
                             const CubicPolynomial& elevation,
                             const CubicPolynomial& superelevation) {
  assert(lane_.get() == nullptr);
  std::unique_ptr<ArcLane> lane = make_unique<ArcLane>(
      id, this, center, radius, theta0, d_theta,
      lane_bounds, driveable_bounds,
      elevation, superelevation);
  ArcLane* result = lane.get();
  lane_ = std::move(lane);
  return result;
}


const api::Lane* Segment::lane(int index) const {
  assert(index == 0);
  return lane_.get();
}




} // namespace monolane
} // namespace maliput

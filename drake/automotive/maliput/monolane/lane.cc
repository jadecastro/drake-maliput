#include "lane.h"

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::Segment* Lane::segment() const { return segment_; }

const api::BranchPoint* Lane::GetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: { return start_bp_; }
    case api::LaneEnd::kEnd:   { return end_bp_; }
  }
  DRAKE_ABORT();
}

const api::SetOfLaneEnds* Lane::GetBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetBranches({this, which_end});
}

const boost::optional<api::LaneEnd>& Lane::GetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}


double Lane::length() const {
  return elevation().s_p(1.0) * p_scale_;
}


Rot3 Lane::rot3_of_p_(const double p) const {
  return Rot3(heading_of_p_(p),
              -std::atan(elevation().fdot_p(p)),
              superelevation().f_p(p) * p_scale_);
}


api::GeoPosition Lane::ToGeoPosition(const api::LanePosition& lane_pos) const {
  // Recover parameter p from arc-length position s.
  const double p = elevation().p_s(lane_pos.s_ / p_scale_);
  // Calculate z (elevation) of (s,0,0);
  const double z = elevation().f_p(p) * p_scale_;
  // Calculate x,y of (s,0,0).
  const V2 xy = xy_of_p_(p);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = rot3_of_p_(p);

  // Rotate (0,r,h) and sum with mapped (s,0,0).
  const V3 xyz = V3::sum(ypr.apply({0., lane_pos.r_, lane_pos.h_}),
                         {xy.x, xy.y, z});
  return {xyz.x, xyz.y, xyz.z};
}

api::Rotation Lane::GetOrientation(const api::LanePosition& lane_pos) const {
  // Recover linear parameter p from arc-length position s.
  const double p = elevation().p_s(lane_pos.s_ / p_scale_);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = rot3_of_p_(p);
  return api::Rotation(ypr.roll,
                       ypr.pitch,
                       ypr.yaw);
}



} // namespace monolane
} // namespace maliput

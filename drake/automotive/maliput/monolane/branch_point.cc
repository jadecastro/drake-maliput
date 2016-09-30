#include "branch_point.h"

#include "lane.h"

#include "ignore.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::LaneEnd& SetOfLaneEnds::do_get(int index) const {
  return ends_[index]; }

BranchPoint::BranchPoint(const api::BranchPointId& id, RoadGeometry* rg)
      : id_(id), road_geometry_(rg) { ignore(road_geometry_); }

const api::SetOfLaneEnds* BranchPoint::DoGetBranches(
    const api::LaneEnd& end) const {
  return branches_.at(end);
}

const boost::optional<api::LaneEnd>& BranchPoint::DoGetDefaultBranch(
    const api::LaneEnd& end) const {
  return defaults_.find(end)->second;
}

const api::LaneEnd& BranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(branches_.find(lane_end) == branches_.end());
  a_side_.add(lane_end);
  branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& BranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  DRAKE_DEMAND(branches_.find(lane_end) == branches_.end());
  b_side_.add(lane_end);
  branches_[lane_end] = &a_side_;
  return lane_end;
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  DRAKE_DEMAND(branches_.find(lane_end) != branches_.end());
  DRAKE_DEMAND(branches_.find(default_branch) != branches_.end());
  // TODO(maddog)  assert that default_branch is actually legal for lane_end.
  defaults_[lane_end] = default_branch;
}



}  // namespace monolane
}  // namespace maliput

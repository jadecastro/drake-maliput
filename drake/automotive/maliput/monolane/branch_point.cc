#include "branch_point.h"

#include <cassert>

#include "lane.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::LaneEnd& SetOfLaneEnds::get(int index) const { return ends_[index]; }


const api::SetOfLaneEnds* BranchPoint::GetBranches(
    const api::LaneEnd& end) const {
  // TODO maddog  Deal with no-such-key.
  return branches_.at(end);
}

const api::LaneEnd& BranchPoint::GetDefaultBranch(
    const api::LaneEnd& end) const {
  // TODO maddog  Deal with no-such-key.
  return defaults_.at(end);
}

const api::LaneEnd& BranchPoint::AddABranch(const api::LaneEnd& lane_end) {
  assert(branches_.find(lane_end) == branches_.end());
  a_side_.add(lane_end);
  branches_[lane_end] = &b_side_;
  return lane_end;
}

const api::LaneEnd& BranchPoint::AddBBranch(const api::LaneEnd& lane_end) {
  assert(branches_.find(lane_end) == branches_.end());
  b_side_.add(lane_end);
  branches_[lane_end] = &a_side_;
  return lane_end;
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end,
                             const api::LaneEnd& default_branch) {
  assert(branches_.find(lane_end) != branches_.end());
  assert(branches_.find(default_branch) != branches_.end());
  // TODO(maddog)  assert that default_branch is an option for lane_end.
  defaults_[lane_end] = default_branch;
}



} // namespace monolane
} // namespace maliput

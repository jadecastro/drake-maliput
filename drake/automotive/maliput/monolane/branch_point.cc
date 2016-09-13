#include "branch_point.h"

#include "lane.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::LaneEnd& SetOfLaneEnds::get(int index) const { return ends_[index]; }


const api::SetOfLaneEnds* BranchPoint::GetBranches(
    const api::LaneEnd& end) const {
  // TODO maddog  Deal with no-such-key.
  return &(branches_.at(end));
}

const api::LaneEnd& BranchPoint::GetDefaultBranch(
    const api::LaneEnd& end) const {
  // TODO maddog  Deal with no-such-key.
  return defaults_.at(end);
}



} // namespace monolane
} // namespace maliput

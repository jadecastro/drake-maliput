#include "branch_point.h"

#include "lane.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::Lane* SetOfLanes::get(int index) const { return lanes_[index]; }


const api::SetOfLanes* BranchPoint::GetBranches(
    const api::Lane* lane,
    const api::Lane::Endpoint which_end) const {
  // TODO maddog  Deal with no-such-key.
  return &(branches_.at(EndedLane(lane, which_end)));
}

const api::Lane* BranchPoint::GetDefaultBranch(
    const api::Lane* lane,
    const api::Lane::Endpoint which_end) const {
  // TODO maddog  Deal with no-such-key.
  return defaults_.at(EndedLane(lane, which_end));
}



} // namespace monolane
} // namespace maliput

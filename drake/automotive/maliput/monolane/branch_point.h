#pragma once

#include <map>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace monolane {

class BranchPoint;
class Lane;
class RoadGeometry;


class DRAKE_EXPORT SetOfLaneEnds : public api::SetOfLaneEnds {
 public:
  virtual ~SetOfLaneEnds() {}

  void add(const api::LaneEnd& end) { ends_.push_back(end); }

 private:
  int do_count() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override;

  std::vector<api::LaneEnd> ends_;
};


class DRAKE_EXPORT BranchPoint : public api::BranchPoint {
 public:
  BranchPoint(const api::BranchPointId& id, RoadGeometry* rg);

  const api::LaneEnd& AddABranch(const api::LaneEnd& lane_end);

  const api::LaneEnd& AddBBranch(const api::LaneEnd& lane_end);

  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

 private:
  const api::BranchPointId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  const api::SetOfLaneEnds* DoGetConfluentBranches(
      const api::LaneEnd& end) const override;

  const api::SetOfLaneEnds* DoGetOngoingBranches(
      const api::LaneEnd& end) const override;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::SetOfLaneEnds* DoGetASide() const override { return &a_side_; }

  const api::SetOfLaneEnds* DoGetBSide() const override { return &b_side_; }

  api::BranchPointId id_;
  RoadGeometry* road_geometry_{};
  SetOfLaneEnds a_side_;
  SetOfLaneEnds b_side_;

  std::map<api::LaneEnd, SetOfLaneEnds*,
           api::LaneEnd::StrictOrder> confluent_branches_;
  std::map<api::LaneEnd, SetOfLaneEnds*,
           api::LaneEnd::StrictOrder> ongoing_branches_;
  std::map<api::LaneEnd, api::LaneEnd,
           api::LaneEnd::StrictOrder> defaults_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake

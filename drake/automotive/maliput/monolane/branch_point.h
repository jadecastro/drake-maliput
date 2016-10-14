#pragma once

#include <map>
#include <vector>

#include "drake/automotive/maliput/geometry_api/branch_point.h"
#include "drake/automotive/maliput/geometry_api/lane.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


class BranchPoint;
class Lane;
class RoadGeometry;


class DRAKEAUTOMOTIVE_EXPORT SetOfLaneEnds : public api::SetOfLaneEnds {
 public:
  virtual ~SetOfLaneEnds() {}

  void add(const api::LaneEnd& end) { ends_.push_back(end); }

 private:
  int do_count() const override { return ends_.size(); }

  const api::LaneEnd& do_get(int index) const override;

  std::vector<api::LaneEnd> ends_;
};


class DRAKEAUTOMOTIVE_EXPORT BranchPoint : public api::BranchPoint {
 public:
  BranchPoint(const api::BranchPointId& id, RoadGeometry* rg);

  const api::LaneEnd& AddABranch(const api::LaneEnd& lane_end);

  const api::LaneEnd& AddBBranch(const api::LaneEnd& lane_end);

  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

 private:
  struct LaneEndStrictOrder {
    bool operator()(const api::LaneEnd& lhs, const api::LaneEnd& rhs) const {
      auto as_tuple = [](const api::LaneEnd& le) {
        return std::tie(le.lane, le.end);
      };
      return as_tuple(lhs) < as_tuple(rhs);
    }
  };

  const api::BranchPointId do_id() const override { return id_; }

  const api::SetOfLaneEnds* DoGetBranches(
      const api::LaneEnd& end) const override;

  const boost::optional<api::LaneEnd>& DoGetDefaultBranch(
      const api::LaneEnd& end) const override;

  const api::SetOfLaneEnds* DoGetASide() const override { return &a_side_; }

  const api::SetOfLaneEnds* DoGetBSide() const override { return &b_side_; }

  api::BranchPointId id_;
  RoadGeometry* road_geometry_;
  SetOfLaneEnds a_side_;
  SetOfLaneEnds b_side_;

  std::map<api::LaneEnd, SetOfLaneEnds*, LaneEndStrictOrder> branches_;
  std::map<api::LaneEnd, boost::optional<api::LaneEnd>,
           LaneEndStrictOrder> defaults_;
};

}  // namespace monolane
}  // namespace maliput

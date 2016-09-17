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

  virtual int count() const { return ends_.size(); }

  virtual const api::LaneEnd& get(int index) const;

  void add(const api::LaneEnd& end) { ends_.push_back(end); }

 private:
  std::vector<api::LaneEnd> ends_;
};


class DRAKEAUTOMOTIVE_EXPORT BranchPoint : public api::BranchPoint {
 public:
  BranchPoint(const api::BranchPointId& id, RoadGeometry* rg)
      : id_(id), road_geometry_(rg) {}


  // Provide persistent ID.
  virtual const api::BranchPointId id() const { return id_; }

  virtual const api::SetOfLaneEnds* GetBranches(
      const api::LaneEnd& end) const;

  virtual const boost::optional<api::LaneEnd>& GetDefaultBranch(
      const api::LaneEnd& end) const;

  virtual const api::SetOfLaneEnds* GetASide() const {
    return &a_side_;
  }

  virtual const api::SetOfLaneEnds* GetBSide() const {
    return &b_side_;
  }

  const api::LaneEnd& AddABranch(const api::LaneEnd& lane_end);

  const api::LaneEnd& AddBBranch(const api::LaneEnd& lane_end);

  void SetDefault(const api::LaneEnd& lane_end,
                  const api::LaneEnd& default_branch);

 private:
  api::BranchPointId id_;
  RoadGeometry* road_geometry_;
  SetOfLaneEnds a_side_;
  SetOfLaneEnds b_side_;

  std::map<api::LaneEnd, SetOfLaneEnds*> branches_;
  std::map<api::LaneEnd, boost::optional<api::LaneEnd>> defaults_;
};



} // namespace monolane
} // namespace maliput

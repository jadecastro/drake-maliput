#pragma once

#include <map>
#include <vector>

#include "geometry_api/branch_point.h"
#include "geometry_api/lane.h"
#include "geometry_api/road_geometry.h"


namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


class BranchPoint;
class Lane;
class RoadGeometry;


class SetOfLaneEnds : public api::SetOfLaneEnds {
 public:
  virtual ~SetOfLaneEnds() {}

  virtual int count() const { return ends_.size(); }

  virtual const api::LaneEnd& get(int index) const;

 private:
  std::vector<api::LaneEnd> ends_;
};


class BranchPoint : public api::BranchPoint {
 public:
  // Provide persistent ID.
  virtual const api::BranchPointId id() const { return id_; }

  virtual const api::SetOfLaneEnds* GetBranches(
      const api::LaneEnd& end) const;

  virtual const api::LaneEnd& GetDefaultBranch(
      const api::LaneEnd& end) const;

  virtual const api::SetOfLaneEnds* GetASide() const {
    return &a_side_;
  }

  virtual const api::SetOfLaneEnds* GetBSide() const {
    return &b_side_;
  }

 private:
  api::BranchPointId id_;
  RoadGeometry* road_geometry_;
  SetOfLaneEnds a_side_;
  SetOfLaneEnds b_side_;

  std::map<api::LaneEnd, SetOfLaneEnds> branches_;
  std::map<api::LaneEnd, api::LaneEnd> defaults_;
};



} // namespace monolane
} // namespace maliput

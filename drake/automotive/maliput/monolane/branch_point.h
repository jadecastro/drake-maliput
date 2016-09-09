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


class SetOfLanes : public api::SetOfLanes {
 public:
  virtual int count() const { return lanes_.size(); }

  virtual const api::Lane* get(int index) const;

 private:
  std::vector<Lane*> lanes_;
};


class BranchPoint : public api::BranchPoint {

 private:
  typedef std::pair<const api::Lane*, api::Lane::Endpoint> EndedLane;

 public:
  // Provide persistent ID.
  virtual const api::BranchPointId id() const { return id_; }

  virtual const api::SetOfLanes* GetBranches(
      const api::Lane* lane,
      const api::Lane::Endpoint which_end) const;

  virtual const api::Lane* GetDefaultBranch(
      const api::Lane* lane,
      const api::Lane::Endpoint which_end) const;

  virtual const api::SetOfLanes* GetASide() const {
    return &a_side_;
  }

  virtual const api::SetOfLanes* GetBSide() const {
    return &b_side_;
  }

 private:
  api::BranchPointId id_;
  SetOfLanes a_side_;
  SetOfLanes b_side_;

  std::map<EndedLane, SetOfLanes*> branches_;
  std::map<EndedLane, Lane*> defaults_;
};



} // namespace monolane
} // namespace maliput

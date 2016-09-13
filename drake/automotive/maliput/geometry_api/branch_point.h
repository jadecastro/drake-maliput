#pragma once

#include "lane.h"

namespace maliput {
namespace geometry_api {


class SetOfLaneEnds {
 public:
  virtual ~SetOfLaneEnds() {}

  // TODO maddog  Improve this.
  virtual int count() const = 0;
  virtual const LaneEnd& get(int index) const = 0;
};



class BranchPoint {
 public:
  virtual ~BranchPoint() {}

  // Provide persistent ID.
  virtual const BranchPointId id() const = 0;

  virtual const SetOfLaneEnds* GetBranches(const LaneEnd& end) const = 0;

  virtual const LaneEnd& GetDefaultBranch(const LaneEnd& end) const = 0;

  virtual const SetOfLaneEnds* GetASide() const = 0;

  virtual const SetOfLaneEnds* GetBSide() const = 0;
};



} // namespace geometry_api
} // namespace maliput

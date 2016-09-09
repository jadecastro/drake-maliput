#pragma once

#include "lane.h"

namespace maliput {
namespace geometry_api {



class SetOfLanes {
 public:
  virtual ~SetOfLanes() {}

  // TODO maddog  Improve this.
  virtual int count() const = 0;
  virtual const Lane* get(int index) const = 0;
};



class BranchPoint {
 public:
  virtual ~BranchPoint() {}

  // Provide persistent ID.
  virtual const BranchPointId id() const = 0;

  virtual const SetOfLanes* GetBranches(const Lane* lane,
                                        const Lane::Endpoint which_end) const = 0;
  virtual const Lane* GetDefaultBranch(const Lane* lane,
                                       const Lane::Endpoint which_end) const = 0;
  virtual const SetOfLanes* GetASide() const = 0;

  virtual const SetOfLanes* GetBSide() const = 0;
};



} // namespace geometry_api
} // namespace maliput

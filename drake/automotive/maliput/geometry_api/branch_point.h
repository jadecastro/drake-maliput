#pragma once

#include "lane.h"

#include <boost/optional.hpp>

namespace maliput {
namespace geometry_api {

class RoadGeometry;

class DRAKE_EXPORT SetOfLaneEnds {
 public:
  virtual ~SetOfLaneEnds() {}

  // TODO(maddog)  Improve this.
  int count() const { return do_count(); }

  const LaneEnd& get(int index) const { return do_get(index); }

 private:
  // TODO(maddog)  Improve this.
  virtual int do_count() const = 0;

  virtual const LaneEnd& do_get(int index) const = 0;
};



class DRAKE_EXPORT BranchPoint {
 public:
  virtual ~BranchPoint() {}

  // Provide persistent ID.
  const BranchPointId id() const { return do_id(); }

  /// @return the RoadGeometry which owns this BranchPoint.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  const SetOfLaneEnds* GetBranches(const LaneEnd& end) const {
    return DoGetBranches(end);
  }

  const boost::optional<LaneEnd>& GetDefaultBranch(const LaneEnd& end) const {
    return DoGetDefaultBranch(end);
  }

  const SetOfLaneEnds* GetASide() const { return DoGetASide(); }

  const SetOfLaneEnds* GetBSide() const { return DoGetBSide(); }

 private:
  // Provide persistent ID.
  virtual const BranchPointId do_id() const = 0;

  /// @return the RoadGeometry which owns this BranchPoint.
  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual const SetOfLaneEnds* DoGetBranches(const LaneEnd& end) const = 0;

  virtual const boost::optional<LaneEnd>& DoGetDefaultBranch(
      const LaneEnd& end) const = 0;

  virtual const SetOfLaneEnds* DoGetASide() const = 0;

  virtual const SetOfLaneEnds* DoGetBSide() const = 0;
};



}  // namespace geometry_api
}  // namespace maliput

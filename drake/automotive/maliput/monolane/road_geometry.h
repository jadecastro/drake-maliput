#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/automotive/maliput/geometry_api/branch_point.h"
#include "drake/automotive/maliput/geometry_api/junction.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;

/// A simple RoadGeometry implementation that only supports a
/// single lane per segment.
class DRAKEAUTOMOTIVE_EXPORT RoadGeometry : public api::RoadGeometry {
 public:
  explicit RoadGeometry(const api::RoadGeometryId& id)
      : id_(id) {}


  Junction* NewJunction(api::JunctionId id);

  BranchPoint* NewBranchPoint(api::BranchPointId id);

  virtual const api::RoadGeometryId id() const {
    return id_;
  }

  virtual int num_junctions() const {
    return junctions_.size();
  }

  virtual const api::Junction* junction(int index) const;

  virtual int num_branch_points() const {
    return branch_points_.size();
  }

  virtual const api::BranchPoint* branch_point(int index) const;

  virtual api::RoadPosition ToRoadPosition(const api::GeoPosition& geo_pos,
                                           const api::RoadPosition& hint) const;

 private:
  api::RoadGeometryId id_;
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
};



}  // namespace monolane
}  // namespace maliput

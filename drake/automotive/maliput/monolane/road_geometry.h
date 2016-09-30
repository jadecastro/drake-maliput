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

 private:
  const api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const override;

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_pos,
      const api::RoadPosition& hint) const override;

  api::RoadGeometryId id_;
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
};

}  // namespace monolane
}  // namespace maliput

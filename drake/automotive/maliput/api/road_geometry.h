#pragma once

#include "drake/automotive/maliput/api/lane_data.h"

#include <vector>

namespace drake {
namespace maliput {
namespace api {

class BranchPoint;
class Junction;


struct DRAKE_EXPORT RoadGeometryId {
  std::string id;
};


/// Abstract API for the geometry of a road network, including both
/// the network topology and the geometry of its embedding in 3-space.
class DRAKE_EXPORT RoadGeometry {
 public:
  virtual ~RoadGeometry() {}

  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  const RoadGeometryId id() const { return do_id(); }

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Junctions in the RoadGeometry.
  int num_junctions() const { return do_num_junctions(); }

  /// @return the Junction indexed by @param index.
  const Junction* junction(int index) const { return do_junction(index); }

  /// @return the number of BrancPoints n the RoadGeometry.
  int num_branch_points() const { return do_num_branch_points(); }

  /// @return the BranchPoint indexed by @param index.
  const BranchPoint* branch_point(int index) const {
    return do_branch_point(index);
  }

  /// Figure out the RoadPosition corresponding to GeoPosition @param geo_pos,
  /// potentially using @param hint to guide the search.
  // TODO(maddog)  UNDER CONSTRUCTION
  RoadPosition ToRoadPosition(const GeoPosition& geo_pos,
                              const RoadPosition& hint) const {
    return DoToRoadPosition(geo_pos, hint);
  }

  /// @return the tolerance guaranteed for linear measurements (positions).
  double linear_tolerance() const {
    return do_linear_tolerance();
  }

  /// @return the tolerance guaranteed for angular measurements (orientations).
  double angular_tolerance() const {
    return do_angular_tolerance();
  }

  /// Verify certain invariants guaranteed by the API.
  ///
  /// @returns a vector of strings describing violations of invariants.
  /// Return value with size() == 0 indicates success.
  std::vector<std::string> CheckInvariants() const;

 private:
  /// @return the persistent identifier.
  ///
  // TODO(maddog)  Tie id into a tiling mechanism?
  virtual const RoadGeometryId do_id() const = 0;

  // TODO(maddog)  Real iteration support over junctions and branch-points.

  /// @return the number of Junctions in the RoadGeometry.
  virtual int do_num_junctions() const = 0;

  /// @return the Junction indexed by @param index.
  virtual const Junction* do_junction(int index) const = 0;

  /// @return the number of BrancPoints n the RoadGeometry.
  virtual int do_num_branch_points() const = 0;

  /// @return the BranchPoint indexed by @param index.
  virtual const BranchPoint* do_branch_point(int index) const = 0;

  /// Figure out the RoadPosition corresponding to GeoPosition @param geo_pos,
  /// potentially using @param hint to guide the search.
  // TODO(maddog)  UNDER CONSTRUCTION
  virtual RoadPosition DoToRoadPosition(const GeoPosition& geo_pos,
                                        const RoadPosition& hint) const = 0;

  /// @return the tolerance guaranteed for linear measurements (positions).
  virtual double do_linear_tolerance() const = 0;

  /// @return the tolerance guaranteed for angular measurements (orientations).
  virtual double do_angular_tolerance() const = 0;
};


}  // namespace api
}  // namespace maliput
}  // namespace drake

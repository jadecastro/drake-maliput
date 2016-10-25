#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/geometry_api/branch_point.h"
#include "drake/automotive/maliput/geometry_api/lane.h"
#include "drake/automotive/maliput/geometry_api/junction.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"
#include "drake/automotive/maliput/geometry_api/segment.h"
#include "drake/automotive/maliput/geometry_api/state.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;

/// A toy adapter class that makes a RoadGeometry look like it has a single
/// inifinitely long Lane.  Its primary (and perhaps only) purpose is to
/// facilitate demos until proper hybrid system support is available.
///
/// Caveats:
///  * Only works with a RoadGeometry that has one-lane-per-segment.
///  * Source RoadGeometry must have no dead-ends.
class DRAKE_EXPORT InfiniteCircuitRoad : public api::RoadGeometry {
 public:
  /// Construct an InfiniteCircuitRoad based on @param source, using
  /// @param start as the starting point in the search for a closed circuit.
  ///
  /// NB:  All the real construction work happens in the constructor for
  /// InfiniteCircuitRoad::Lane.
  InfiniteCircuitRoad(const api::RoadGeometryId& id,
                      const api::RoadGeometry* source,
                      const api::LaneEnd& start,
                      const std::vector<const api::Lane*>& path);

  virtual ~InfiniteCircuitRoad();

  /// @returns the sole Lane component emulated by this RoadGeometry.
  const api::Lane* lane() const { return &lane_; }

  /// @returns the actual length of a single cycle (despite the illusion that
  /// the road is infinitely long).
  double cycle_length() const { return lane_.cycle_length(); }

  /// @returns a pointer to the underlying source RoadGeometry.
  const api::RoadGeometry* source() const { return source_; }

  /// Project the given LanePosition on the "infinite Lane" back to the
  /// source Lane.
  ///
  /// @returns a pair of:
  /// - RoadPosition indicating the source Lane and position on that Lane;
  /// - bool indicating travel should be reversed in the source Lane with
  ///   respect to +s travel in the Infinite Lane --- e.g., if true, then
  ///   +s motion in the Infinite Lane corresponds to -s motion in the
  ///   source Lane.
  std::pair<api::RoadPosition, bool> ProjectToSourceRoad(
      const api::LanePosition& lane_pos) const {
    return lane_.ProjectToSourceRoad(lane_pos);
  }

 private:
  class Junction;
  class Segment;

  class DRAKE_EXPORT Lane : public api::Lane {
   public:
    Lane(const api::LaneId& id, const Segment* segment,
         const api::RoadGeometry* source,
         const api::LaneEnd& start,
         const std::vector<const api::Lane*>& path);

    virtual ~Lane();

    double cycle_length() const { return cycle_length_; }

    std::pair<api::RoadPosition, bool> ProjectToSourceRoad(
        const api::LanePosition& lane_pos) const;

   private:
    const api::LaneId do_id() const override { return id_; }

    const api::Segment* do_segment() const override { return segment_; }

    int do_index() const override { return 0; }  // Only one lane per segment!

    const api::Lane* do_to_left() const override { return nullptr; }

    const api::Lane* do_to_right() const override { return nullptr; }

    const api::BranchPoint* DoGetBranchPoint(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    const api::SetOfLaneEnds* DoGetBranches(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    const boost::optional<api::LaneEnd>& DoGetDefaultBranch(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    double do_length() const override;

    api::RBounds do_lane_bounds(double) const override { return lane_bounds_; }

    api::RBounds do_driveable_bounds(double) const override {
      return driveable_bounds_;
    }

    api::GeoPosition DoToGeoPosition(
        const api::LanePosition& lane_pos) const override;

    api::Rotation DoGetOrientation(
        const api::LanePosition& lane_pos) const override;

    void DoEvalMotionDerivatives(const api::LanePosition& position,
                                 const api::IsoLaneVelocity& velocity,
                                 api::LanePosition* position_dot) const override;

    api::LanePosition DoToLanePosition(
        const api::GeoPosition&) const override {
      DRAKE_ABORT();  // TODO(maddog)  Implement when someone needs this.
    }

    struct Record {
      const api::Lane* lane;
      double start_circuit_s;
      double end_circuit_s;
      bool is_reversed;
    };


    const api::LaneId id_;
    const Segment* segment_;

    api::RBounds lane_bounds_;
    api::RBounds driveable_bounds_;


    std::vector<Record> records_;
    double cycle_length_;
  };


  class DRAKE_EXPORT Segment : public api::Segment {
   public:
    Segment(const api::SegmentId& id,
            const Junction* junction, const Lane* lane)
        : id_(id), junction_(junction), lane_(lane) {}

    virtual ~Segment() {}

   private:
    const api::SegmentId do_id() const override { return id_; }

    const api::Junction* do_junction() const override { return junction_; }

    int do_num_lanes() const override { return 1; }

    const api::Lane* do_lane(int index) const override { return lane_; }

    api::SegmentId id_;
    const Junction* junction_;
    const Lane* lane_;
  };


  class DRAKE_EXPORT Junction : public api::Junction {
   public:
    Junction(const api::JunctionId& id,
             const RoadGeometry* rg, const Segment* segment)
        : id_(id), road_geometry_(rg), segment_(segment) {}

    virtual ~Junction() {}

   private:
    const api::JunctionId do_id() const override { return id_; }

    const api::RoadGeometry* do_road_geometry() const override {
      return road_geometry_;
    }

    int do_num_segments() const override { return 1; }

    const api::Segment* do_segment(int index) const override { return segment_; }

    api::JunctionId id_;
    const RoadGeometry* road_geometry_;
    const Segment* segment_;
  };


  const api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return 1; }

  const api::Junction* do_junction(int index) const override {
    return &junction_;
  }

  int do_num_branch_points() const override { return 0; }

  const api::BranchPoint* do_branch_point(int index) const override {
    DRAKE_ABORT();
  }

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_pos,
      const api::RoadPosition& hint) const override {
    DRAKE_ABORT();  // TODO(maddog)  Implement when someone needs this.
  }

  double do_linear_tolerance() const override {
    return source_->linear_tolerance();
  }

  double do_angular_tolerance() const override {
    return source_->angular_tolerance();
  }

  const api::RoadGeometryId id_;
  const api::RoadGeometry* const source_;
  const Junction junction_;
  const Segment segment_;
  const Lane lane_;
};

}  // namespace utility
}  // namespace maliput

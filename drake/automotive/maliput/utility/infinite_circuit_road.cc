#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

#include <cmath>

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/ignore.h"
#include "drake/automotive/maliput/monolane/junction.h"

#include "drake/common/drake_assert.h"


namespace maliput {
namespace utility {

namespace api = maliput::geometry_api;

InfiniteCircuitRoad::InfiniteCircuitRoad(const api::RoadGeometryId& id,
                                         const api::RoadGeometry* source,
                                         const api::LaneEnd& start)
    : id_(id),
      junction_({id.id_ + ".junction"}, this, &segment_),
      segment_({id.id_ + ".segment"}, &junction_, &lane_),
      lane_({id.id_ + ".lane"}, &segment_, source, start) {}


InfiniteCircuitRoad::~InfiniteCircuitRoad() {}



InfiniteCircuitRoad::Lane::Lane(const api::LaneId& id,
                                const Segment* segment,
                                const api::RoadGeometry* source,
                                const api::LaneEnd& start)
    : id_(id),
      segment_(segment) {
  // Starting at start, walk source's Lane/BranchPoint graph.  We assume
  // that there are no dead-end branch-points, so we will eventually
  // encounter a Lane that we have seen before, at which point we know
  // that we have found a cycle.  Along the way, we will keep track of
  // accumulated s-length over the sequence of lanes.

  std::map<const api::Lane*, int> seen_lane_index;
  double start_s = 0.;
  api::LaneEnd current = start;

  while (!seen_lane_index.count(current.lane_)) {
    const double end_s = start_s + current.lane_->length();
    seen_lane_index[current.lane_] = records_.size();
    records_.push_back(Record {
        current.lane_, start_s, end_s, (current.end_ == api::LaneEnd::kEnd)});

    const api::SetOfLaneEnds* branches =
        current.lane_->GetBranches(current.end_);
    DRAKE_DEMAND(branches->count() > 0);
    // Use the first branch every time == simple.
    current = branches->get(0);

    start_s = end_s;
  }

  // TODO(maddog)  For now, just assert that we came back to where we started.
  //               (Otherwise, we have to trim records off the front, and
  //               adjust the total length.)
  DRAKE_DEMAND((current.lane_ == start.lane_) &&
               (current.end_ == start.end_));
  cycle_length_ = start_s;
}


InfiniteCircuitRoad::Lane::~Lane() {}


double InfiniteCircuitRoad::Lane::do_length() const {
  return INFINITY;
}

api::GeoPosition
InfiniteCircuitRoad::Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  const api::RoadPosition rp = ProjectToSourceRoad(lane_pos).first;
  return rp.lane_->ToGeoPosition(rp.pos_);
}


api::Rotation InfiniteCircuitRoad::Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad(lane_pos);
  api::Rotation result = rp.lane_->GetOrientation(rp.pos_);
  if (is_reversed) {
    result.roll_ = -result.roll_;
    result.pitch_ = -result.pitch_;
    result.yaw_ = result.yaw_ + M_PI;
  }
  return result;
}


void InfiniteCircuitRoad::Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity,
    api::LanePosition* position_dot) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad(position);
  rp.lane_->EvalMotionDerivatives(
      rp.pos_,
      is_reversed ? api::IsoLaneVelocity(-velocity.sigma_v_,
                                         -velocity.rho_v_,
                                         velocity.eta_v_) : velocity,
      position_dot);
}


std::pair<api::RoadPosition, bool>
InfiniteCircuitRoad::Lane::ProjectToSourceRoad(
    const api::LanePosition& lane_pos) const {
  // Find phase within the circuit.
  // TODO(maddog)  Yes, this has obvious precision problems as lane_pos.s_
  //               grows without bounds.
  double circuit_s = std::fmod(lane_pos.s_, cycle_length_);
  if (circuit_s < 0.) { circuit_s += cycle_length_; }

  for (const Record& r : records_) {
    if (circuit_s < r.end_circuit_s) {
      const double s_offset = circuit_s - r.start_circuit_s;
      if (r.is_reversed) {
        // If this Lane is connected "backwards", then we have to measure s
        // from the end, and flip the sign of r.
        return std::make_pair(
            api::RoadPosition(
                r.lane, api::LanePosition(r.lane->length() - s_offset,
                                          -lane_pos.r_,
                                          lane_pos.h_)),
            true /*reversed*/);
      } else {
        return std::make_pair(
            api::RoadPosition(
                r.lane, api::LanePosition(s_offset, lane_pos.r_, lane_pos.h_)),
            false /*not reversed*/);
      }
    }
  }
  DRAKE_ABORT(); // I.e., how did we fall off the end?
}

}  // namespace monolane
}  // namespace maliput

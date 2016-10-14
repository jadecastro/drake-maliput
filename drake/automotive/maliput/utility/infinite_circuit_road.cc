#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

#include <cmath>
#include <iostream>

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
      junction_({id.id + ".junction"}, this, &segment_),
      segment_({id.id + ".segment"}, &junction_, &lane_),
      lane_({id.id + ".lane"}, &segment_, source, start) {}


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

  while (!seen_lane_index.count(current.lane)) {
    std::cerr << "walk lane " << current.lane->id().id
              << "  end " << current.end
              << "   length " << current.lane->length() << std::endl;
    const double end_s = start_s + current.lane->length();
    seen_lane_index[current.lane] = records_.size();
    records_.push_back(Record {
        current.lane, start_s, end_s, (current.end == api::LaneEnd::kFinish)});

    api::LaneEnd::Which other_end =
        (current.end == api::LaneEnd::kStart) ?
        api::LaneEnd::kFinish :
        api::LaneEnd::kStart;
    const api::SetOfLaneEnds* branches = current.lane->GetBranches(other_end);
    DRAKE_DEMAND(branches->count() > 0);
    // Use the first branch every time == simple.
    current = branches->get(0);
    std::cerr << branches->count() << " branches, "
              << " 0 ---> lane " << current.lane->id().id
              << ", end " << current.end
              << std::endl;

    start_s = end_s;
  }

  // TODO(maddog)  For now, just assert that we came back to where we started.
  //               (Otherwise, we have to trim records off the front, and
  //               adjust the total length.)
  DRAKE_DEMAND((current.lane == start.lane) &&
               (current.end == start.end));
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
  return rp.lane->ToGeoPosition(rp.pos);
}


api::Rotation InfiniteCircuitRoad::Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  api::RoadPosition rp;
  bool is_reversed;
  std::tie(rp, is_reversed) = ProjectToSourceRoad(lane_pos);
  api::Rotation result = rp.lane->GetOrientation(rp.pos);
  if (is_reversed) {
    result.roll = -result.roll;
    result.pitch = -result.pitch;
    result.yaw = result.yaw + M_PI;
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
  rp.lane->EvalMotionDerivatives(
      rp.pos,
      is_reversed ? api::IsoLaneVelocity(-velocity.sigma_v,
                                         -velocity.rho_v,
                                         velocity.eta_v) : velocity,
      position_dot);
}


std::pair<api::RoadPosition, bool>
InfiniteCircuitRoad::Lane::ProjectToSourceRoad(
    const api::LanePosition& lane_pos) const {
  // Find phase within the circuit.
  // TODO(maddog)  Yes, this has obvious precision problems as lane_pos.s_
  //               grows without bounds.
  double circuit_s = std::fmod(lane_pos.s, cycle_length_);
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
                                          -lane_pos.r,
                                          lane_pos.h)),
            true /*reversed*/);
      } else {
        return std::make_pair(
            api::RoadPosition(
                r.lane, api::LanePosition(s_offset, lane_pos.r, lane_pos.h)),
            false /*not reversed*/);
      }
    }
  }
  DRAKE_ABORT(); // I.e., how did we fall off the end?
}

}  // namespace monolane
}  // namespace maliput

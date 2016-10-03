#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

#include "arc_lane.h"
#include "branch_point.h"
#include "ignore.h"
#include "line_lane.h"
#include "road_geometry.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

Builder::XYZPointFuzzyOrder::
XYZPointFuzzyOrder(const double position_precision,
                   const double orientation_precision)
        : pos_pre_(position_precision),
          ori_pre_(orientation_precision) {
      ignore(pos_pre_);
      ignore(ori_pre_);
    }

Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds,
                 const double position_precision,
                 const double orientation_precision)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds),
      position_precision_(position_precision),
      orientation_precision_(orientation_precision) {}

const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const double length,
    const ZPoint& z_end) {

  const XYZPoint end(
      XYPoint(start.xy_.x_ + (length * std::cos(start.xy_.heading_)),
              start.xy_.y_ + (length * std::sin(start.xy_.heading_)),
              start.xy_.heading_),
      z_end);
  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kLine, id,
      start, end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const double length,
    const XYZPoint& explicit_end) {
  DRAKE_DEMAND(length);  // TODO(maddog)  Validate length vs forced_end.
  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kLine, id,
      start, explicit_end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const ZPoint& z_end) {
  const double alpha = start.xy_.heading_;
  const double theta0 = alpha - std::copysign(M_PI / 2., arc.d_theta_);
  const double theta1 = theta0 + arc.d_theta_;

  const double cx = start.xy_.x_ - (arc.radius_ * std::cos(theta0));
  const double cy = start.xy_.y_ - (arc.radius_ * std::sin(theta0));

  const XYZPoint end(XYPoint(cx + (arc.radius_ * std::cos(theta1)),
                             cy + (arc.radius_ * std::sin(theta1)),
                             alpha + arc.d_theta_),
                     z_end);

  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kArc, id,
      start, end, cx, cy, arc.radius_, arc.d_theta_));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const XYZPoint& explicit_end) {
  const double alpha = start.xy_.heading_;
  const double theta0 = alpha - std::copysign(M_PI / 2., arc.d_theta_);

  const double cx = start.xy_.x_ - (arc.radius_ * std::cos(theta0));
  const double cy = start.xy_.y_ - (arc.radius_ * std::sin(theta0));

  connections_.push_back(std::make_unique<Connection>(
      Connection::Type::kArc, id,
      start, explicit_end, cx, cy, arc.radius_, arc.d_theta_));
  return connections_.back().get();
}


void Builder::SetDefaultBranch(
    const Connection* in, const api::LaneEnd::Which in_end,
    const Connection* out, const api::LaneEnd::Which out_end) {
  default_branches_.push_back({in, in_end, out, out_end});
}


Group* Builder::MakeGroup(const std::string& id) {
  groups_.push_back(std::make_unique<Group>(id));
  return groups_.back().get();
}


Group* Builder::MakeGroup(const std::string& id,
                          const std::vector<const Connection*>& connections) {
  groups_.push_back(std::make_unique<Group>(id, connections));
  return groups_.back().get();
}


namespace {
CubicPolynomial MakeCubic(const double dX, const double Y0, const double dY,
                          const double Ydot0, const double Ydot1) {
  return CubicPolynomial(Y0 / dX,
                         Ydot0,
                         (3. * dY / dX) - (2. * Ydot0) - Ydot1,
                         Ydot0 + Ydot1 - (2. * dY / dX));
}

/// Determine the heading (in xy-plane) along the centerline when
/// travelling towards/into the lane, from the specified end.
double HeadingIntoLane(const api::Lane* const lane,
                       const api::LaneEnd::Which end) {
  switch (end) {
    case api::LaneEnd::kStart: {
      return lane->GetOrientation({0., 0., 0.}).yaw_;
    }
    case api::LaneEnd::kEnd: {
      return -lane->GetOrientation({lane->length(), 0., 0.}).yaw_;
    }
    default: { DRAKE_ABORT(); }
  }
}
}  // namespace



BranchPoint* Builder::FindOrCreateBranchPoint(
    const XYZPoint& point,
    RoadGeometry* rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog) Generate a real id.
  BranchPoint* bp = rg->NewBranchPoint(
      {"bp:" + std::to_string(rg->num_branch_points())});
  auto result = bp_map->emplace(point, bp);
  ignore(result);
  DRAKE_DEMAND(result.second);
  return bp;
}


void Builder::AttachBranchPoint(
    const XYZPoint& point, Lane* const lane, const api::LaneEnd::Which end,
    RoadGeometry* rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const {
  BranchPoint* bp = FindOrCreateBranchPoint(point, rg, bp_map);
  // Is this the first lane-end added to the branch-point?
  // If so, just stick it on Side-A.
  // (NB: We just test size of A-Side, since A-Side is always populated first.)
  if (bp->GetASide()->count() == 0) {
    bp->AddABranch({lane, end});
    return;
  }
  // Otherwise, assess if this new lane-end is parallel or
  // anti-parallel to the first lane-end.  Parallel: go to same,
  // A-side; anti-parallel: other, B-side.  Do this by examining
  // dot-product of heading vectors (rather than goofing around with
  // cyclic angle arithmetic).
  const double new_h = HeadingIntoLane(lane, end);
  const api::LaneEnd old_le = bp->GetASide()->get(0);
  const double old_h = HeadingIntoLane(old_le.lane_, old_le.end_);
  if (((std::cos(new_h) * std::cos(old_h)) +
       (std::sin(new_h) * std::cos(old_h))) > 0.) {
    bp->AddABranch({lane, end});
  } else {
    bp->AddBBranch({lane, end});
  }
  // Make sure the lane knows its branch-points, too!
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBp(bp);
      break;
    }
    case api::LaneEnd::kEnd: {
      lane->SetEndBp(bp);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
}



Lane* Builder::BuildConnection(
    const Connection* const cnx,
    Junction* const junction,
    RoadGeometry* const rg,
    std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const {
  Segment* segment = junction->NewSegment({std::string("s:") + cnx->id()});
  Lane* lane;
  api::LaneId lane_id({std::string("l:") + cnx->id()});

  switch (cnx->type()) {
    case Connection::kLine: {
      const V2 xy0(cnx->start().xy_.x_,
                   cnx->start().xy_.y_);
      const V2 dxy(cnx->end().xy_.x_ - xy0.x,
                   cnx->end().xy_.y_ - xy0.y);
      const CubicPolynomial elevation(MakeCubic(
          dxy.length(),
          cnx->start().z_.z_,
          cnx->end().z_.z_ - cnx->start().z_.z_,
          cnx->start().z_.zdot_,
          cnx->end().z_.zdot_));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.length(),
          cnx->start().z_.theta_,
          cnx->end().z_.theta_ - cnx->start().z_.theta_,
          cnx->start().z_.thetadot_,
          cnx->end().z_.thetadot_));

      lane = segment->NewLineLane(lane_id,
                                  xy0, dxy,
                                  lane_bounds_, driveable_bounds_,
                                  elevation, superelevation);
      break;
    }
    case Connection::kArc: {
      const V2 center(cnx->cx(), cnx->cy());
      const double radius = cnx->radius();
      const double theta0 = std::atan2(cnx->start().xy_.y_ - center.y,
                                       cnx->start().xy_.x_ - center.x);
      // TODO(maddog) Uh, which direction?  Info lost since cnx constructed!
      //              ...and deal with wrap-arounds, too.
      const double d_theta = cnx->d_theta();
      const double arc_length = radius * std::abs(d_theta);
      const CubicPolynomial elevation(MakeCubic(
          arc_length,
          cnx->start().z_.z_,
          cnx->end().z_.z_ - cnx->start().z_.z_,
          cnx->start().z_.zdot_,
          cnx->end().z_.zdot_));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length,
          cnx->start().z_.theta_,
          cnx->end().z_.theta_ - cnx->start().z_.theta_,
          cnx->start().z_.thetadot_,
          cnx->end().z_.thetadot_));

      lane = segment->NewArcLane(lane_id,
                                 center, radius, theta0, d_theta,
                                 lane_bounds_, driveable_bounds_,
                                 elevation, superelevation);
      break;
    }
    default: {
      DRAKE_ABORT();
    }
  }

  AttachBranchPoint(cnx->start(), lane, api::LaneEnd::kStart, rg, bp_map);
  AttachBranchPoint(cnx->end(), lane, api::LaneEnd::kEnd, rg, bp_map);
  return lane;
}


std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {
  auto rg = std::make_unique<RoadGeometry>(id);
  std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder> bp_map(
      XYZPointFuzzyOrder(position_precision_, orientation_precision_));
  std::map<const Connection*, Lane*> lane_map;
  std::set<const Connection*> remaining_connections;

  for (auto& cnx : connections_) {
    remaining_connections.insert(cnx.get());
  }

  for (auto& grp : groups_) {
    Junction* junction = rg->NewJunction({std::string("j:") + grp->id()});
    std::cout << "jnx: " << junction->id().id_ << std::endl;
    for (auto& cnx : grp->connections()) {
      std::cout << "cnx: " << cnx->id() << std::endl;
      remaining_connections.erase(cnx);
      lane_map[cnx] = BuildConnection(cnx, junction, rg.get(), &bp_map);
    }
  }

  for (auto& cnx : remaining_connections) {
    Junction* junction = rg->NewJunction({std::string("j:") + cnx->id()});
    std::cout << "jnx: " << junction->id().id_ << std::endl;
    std::cout << "cnx: " << cnx->id() << std::endl;
    lane_map[cnx] = BuildConnection(cnx, junction, rg.get(), &bp_map);
  }

  for (auto& def : default_branches_) {
    Lane* in_lane = lane_map[def.in_];
    Lane* out_lane = lane_map[def.out_];
    DRAKE_DEMAND((def.in_end_ == api::LaneEnd::kStart) ||
                 (def.in_end_ == api::LaneEnd::kEnd));
    ((def.in_end_ == api::LaneEnd::kStart) ?
     in_lane->start_bp() : in_lane->end_bp())
        ->SetDefault({in_lane, def.in_end_},
                     {out_lane, def.out_end_});
  }

  return rg;
}


}  // namespace monolane
}  // namespace maliput

#include "builder.h"

#include <cmath>
#include <iostream>

#include "arc_lane.h"
#include "branch_point.h"
#include "line_lane.h"
#include "make_unique.h"
#include "road_geometry.h"

namespace maliput {
namespace monolane {

const double kPi = 3.14159;


Builder::Builder(const api::RBounds& lane_bounds,
                 const api::RBounds& driveable_bounds)
    : lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds) {}


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
  connections_.push_back(make_unique<Connection>(
      Connection::Type::kLine, id,
      start, end));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const ZPoint& z_end) {
  const double alpha = start.xy_.heading_;
  const double theta0 = alpha - std::copysign(kPi / 2., arc.d_theta_);
  const double theta1 = theta0 + arc.d_theta_;

  const double cx = start.xy_.x_ - (arc.radius_ * std::cos(theta0));
  const double cy = start.xy_.y_ - (arc.radius_ * std::sin(theta0));

  const XYZPoint end(XYPoint(cx + (arc.radius_ * std::cos(theta1)),
                             cy + (arc.radius_ * std::sin(theta1)),
                             alpha + arc.d_theta_),
                     z_end);

  connections_.push_back(make_unique<Connection>(
      Connection::Type::kArc, id,
      start, end, cx, cy, arc.radius_, arc.d_theta_));
  return connections_.back().get();
}


const Connection* Builder::Connect(
    const std::string& id,
    const XYZPoint& start,
    const ArcOffset& arc,
    const ZPoint& z_end,
    const XYPoint& forced_end) {
  const double alpha = start.xy_.heading_;
  const double theta0 = alpha - std::copysign(kPi / 2., arc.d_theta_);

  const double cx = start.xy_.x_ - (arc.radius_ * std::cos(theta0));
  const double cy = start.xy_.y_ - (arc.radius_ * std::sin(theta0));

  const XYZPoint end(forced_end,
                     z_end);

  connections_.push_back(make_unique<Connection>(
      Connection::Type::kArc, id,
      start, end, cx, cy, arc.radius_, arc.d_theta_));
  return connections_.back().get();
}


namespace {
BranchPoint* FindOrCreateBranchPoint(
    const XYZPoint& point,
    RoadGeometry* rg,
    std::map<XYZPoint, BranchPoint*>* bp_map) {
  auto ibp = bp_map->find(point);
  if (ibp != bp_map->end()) {
    return ibp->second;
  }
  // TODO(maddog) Generate a real id.
  BranchPoint* bp = rg->NewBranchPoint({"xxx"});
  auto result = bp_map->emplace(point, bp);
  assert(result.second);
  return bp;
}


CubicPolynomial MakeCubic(const double dX, const double Y0, const double dY,
                          const double Ydot0, const double Ydot1) {
  return CubicPolynomial(Y0 / dX,
                         Ydot0,
                         (3. * dY / dX) - (2. * Ydot0) - Ydot1,
                         Ydot0 + Ydot1 - (2. * dY / dX));
}
} // namespace


std::unique_ptr<const api::RoadGeometry> Builder::Build(
    const api::RoadGeometryId& id) const {

  auto rg = make_unique<RoadGeometry>(id);

  std::map<XYZPoint, BranchPoint*> bp_map;

  for (auto& cnx: connections_) {
    std::cout << "cnx: " << cnx->id() << std::endl;
    Segment* segment = rg->NewJunction({std::string("j:") + cnx->id()})
        ->NewSegment({std::string("s:") + cnx->id()});
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
        assert(false);
      }
    }

    // 'start' ends by default plug onto 'A' side of branch-points.
    BranchPoint* bp0 = FindOrCreateBranchPoint(cnx->start(), rg.get(), &bp_map);
    bp0->AddABranch(api::LaneEnd(lane, api::LaneEnd::kStart));

    // 'end' ends by default plug onto 'B' side of branch-points.
    BranchPoint* bp1 = FindOrCreateBranchPoint(cnx->end(), rg.get(), &bp_map);
    bp1->AddBBranch(api::LaneEnd(lane, api::LaneEnd::kEnd));
  }

  return rg;
}


} // namespace monolane
} // namespace maliput

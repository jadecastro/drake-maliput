#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>

#include "drake/automotive/maliput/geometry_api/state.h"
#include "drake/common/drake_assert.h"

#include "drake/automotive/maliput/monolane/junction.h"

namespace maliput {

namespace geometry_api {
class RoadGeometry;
}

namespace monolane {

namespace api = maliput::geometry_api;


/// Builder for monolane road networks.
///
/// monolane is a simple, highly-constrained network:
///  - single lane per segment.
///  - constant lane_bounds and driveable_bounds, same for all lanes
///  - only linear and constant-curvature-arc primitives in XY-plane
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation


struct XYPoint {
  XYPoint() {}

  XYPoint(double x, double y, double heading)
      :x_(x), y_(y), heading_(heading) {}


  XYPoint reverse() const {
    return XYPoint(x_, y_,
                   std::atan2(-std::sin(heading_), -std::cos(heading_)));
  }

  double x_{};
  double y_{};
  double heading_{};  // radians, zero == x direction
};


struct ZPoint {
  ZPoint reverse() const {
    return {z_, -zdot_, -theta_, -thetadot_};
  }

  double z_;
  double zdot_;

  double theta_;  // superelevation
  double thetadot_;
};


struct XYZPoint {
  XYZPoint() {}

  XYZPoint(const XYPoint& xy, const ZPoint& z) : xy_(xy), z_(z) {}

  XYZPoint reverse() const {
    return {xy_.reverse(), z_.reverse()};
  }

  XYPoint xy_;
  ZPoint z_;
};


// radius_ must be non-negative.
// d_theta_ > 0. is counterclockwise ('veer to left').
// d_theta_ < 0. is clockwise ('veer to right').
struct ArcOffset {
  ArcOffset() {}

  ArcOffset(const double radius, const double d_theta)
      : radius_(radius), d_theta_(d_theta) {
    DRAKE_DEMAND(radius_ > 0.);
  }

  double radius_{};
  double d_theta_{};
};


class DRAKEAUTOMOTIVE_EXPORT Connection : boost::noncopyable {
 public:
  enum Type { kLine, kArc };

  Connection(const Type type, const std::string& id,
             const XYZPoint& start, const XYZPoint& end)
      : type_(type), id_(id), start_(start), end_(end) {}

  Connection(const Type type, const std::string& id,
             const XYZPoint& start, const XYZPoint& end,
             double cx, double cy, double radius, double d_theta)
      : type_(type), id_(id), start_(start), end_(end),
        cx_(cx), cy_(cy), radius_(radius), d_theta_(d_theta) {}

  Type type() const { return type_; }

  const std::string& id() const { return id_; }

  const XYZPoint& start() const { return start_; }

  const XYZPoint& end() const { return end_; }

  double cx() const { return cx_; }

  double cy() const { return cy_; }

  double radius() const { return radius_; }

  double d_theta() const { return d_theta_; }

  ~Connection() {}

 private:
  Type type_{};
  std::string id_;
  XYZPoint start_;
  XYZPoint end_;

  // Bits specific to type_ == kArc:
  double cx_{};
  double cy_{};
  double radius_{};
  double d_theta_{};
};


class DRAKEAUTOMOTIVE_EXPORT Group : boost::noncopyable {
 public:
  explicit Group(const std::string& id) : id_(id) {}

  Group(const std::string& id,
        const std::vector<const Connection*>& connections)
      : id_(id), connections_(connections) {}

  void Add(Connection* connection);

  const std::string& id() const { return id_; }

  const std::vector<const Connection*>& connections() const {
    return connections_;
  }

 private:
  std::string id_;
  std::vector<const Connection*> connections_;
};


class DRAKEAUTOMOTIVE_EXPORT Builder : boost::noncopyable {
 public:
  Builder(const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds,
          const double position_precision,
          const double orientation_precision);

  // TODO(maddog) Provide for explicit branch-point siding of ends...
  //              e.g. to allow 3-way intersection.
  // TODO(maddog) Handle setting default branch-points.

  // Connect a start point to an end point relative to the start,
  // with a linear displacement.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const double length,
      const ZPoint& z_end);

  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const double length,
      const XYZPoint& explicit_end);

  // Connect a start point to an end point relative to the start,
  // with an arc displacement.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const ArcOffset& arc,
      const ZPoint& z_end);

  // Hackery until generic Connect(start, end) implemented.
  const Connection* Connect(
      const std::string& id,
      const XYZPoint& start,
      const ArcOffset& arc,
      const XYZPoint& explicit_end);

  void SetDefaultBranch(
      const Connection* in, const api::LaneEnd::Which in_end,
      const Connection* out, const api::LaneEnd::Which out_end);

  Group* MakeGroup(const std::string& id);

  Group* MakeGroup(const std::string& id,
                   const std::vector<const Connection*>& connections);

  // Produce a RoadGeometry.
  std::unique_ptr<const api::RoadGeometry> Build(
      const api::RoadGeometryId& id) const;

 private:
  class XYZPointFuzzyOrder {
   public:
    XYZPointFuzzyOrder(const double position_precision,
                       const double orientation_precision);

    // TODO(maddog) This should perhaps incorporate heading/anti-heading.
    // TODO(maddog) This should perhaps incorporate position_precision.
    bool operator()(const XYZPoint& lhs, const XYZPoint& rhs) const {
      auto as_tuple = [](const XYZPoint& p) {
        return std::tie(p.xy_.x_, p.xy_.y_, p.z_.z_);
      };
      return as_tuple(lhs) < as_tuple(rhs);
    }

   private:
    const double pos_pre_;
    const double ori_pre_;
  };

  struct DefaultBranch {
    DefaultBranch() {}

    DefaultBranch(
        const Connection* in, const api::LaneEnd::Which in_end,
        const Connection* out, const api::LaneEnd::Which out_end)
        : in_(in), in_end_(in_end), out_(out), out_end_(out_end) {}

    const Connection* in_;
    api::LaneEnd::Which in_end_;
    const Connection* out_;
    api::LaneEnd::Which out_end_;
  };

  Lane* BuildConnection(
      const Connection* const cnx,
      Junction* const junction,
      RoadGeometry* const rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const;

  BranchPoint* FindOrCreateBranchPoint(
      const XYZPoint& point,
      RoadGeometry* rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* const bp_map) const;

  void AttachBranchPoint(
      const XYZPoint& point, Lane* const lane, const api::LaneEnd::Which end,
      RoadGeometry* rg,
      std::map<XYZPoint, BranchPoint*, XYZPointFuzzyOrder>* bp_map) const;

  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  double position_precision_;
  double orientation_precision_;
  std::vector<std::unique_ptr<Connection>> connections_;
  std::vector<DefaultBranch> default_branches_;
  std::vector<std::unique_ptr<Group>> groups_;
};



}  // namespace monolane
}  // namespace maliput

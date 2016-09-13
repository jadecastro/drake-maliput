#pragma once

namespace maliput {
namespace monolane {


/// Builder for monolane road networks.
///
/// monolane is a simple, highly-constrained network:
///  - single lane per segment.
///  - constant lane_bounds and driveable_bounds, same for all lanes
///  - only linear and constant-curvature-arc primitives in XY-plane
///  - cubic polynomials (parameterized on XY-arc-length) for elevation
///    and superelevation



//XXXstruct LaneParams {
//XXX  double r_min_;
//XXX  double r_center_;
//XXX  double r_max_;
//XXX};




typedef std::vector<std::pair<std::vector<int>,
                              std::vector<int>>> LaneMap;


class Connection : boost::noncopyable {
 public:
  Connection(Builder* builder,
             const std::string& id,
             const Point& start, const Point& end,
             const LaneMap& lane_map);

  const Point& start() const { return start_; }

  const Point& end() const { return end_; }

  const LaneMap& lane_map() const { return lane_map_; }

 private:
  std::string id_;
  Point start_;
  Point end_;
  LaneMap lane_map_;
};


struct XYPoint {
  double x_;
  double y_;
  double heading_; // radians, zero == x direction
};

struct ZPoint {
  double z_;
  double zdot_;

  double theta_;  // superelevation
  double thetadot_;
};

struct XYZPoint {
  XYPoint xy_;
  ZPoint z_;
};

struct ArcOffset {
  double radius_;
  double theta_;
};


class Builder : boost::noncopyable {
 public:
  Builder(const api::RBounds& lane_bounds,
          const api::RBounds& driveable_bounds);

  // Connect a start point to a specific end point.
  const Connection* Connect(
      const std::string& id,
      const Point& start,
      const Point& end,
      const LaneMap& lane_map);

  // Connect a start point to an end point relative to the start,
  // with a linear displacement.
  const Connection* Connect(
      const std::string& id,
      const Point& start,
      const double length,
      const ZPoint& z_end,
      const LaneMap& lane_map);

  // Connect a start point to an end point relative to the start,
  // with an arc displacement.
  const Connection* Connect(
      const std::string& id,
      const Point& start,
      const ArcOffset& arc,
      const ZPoint& z_end,
      const LaneMap& lane_map);

  // Produce a RoadGeometry.
  const RoadGeometry* Build() const;


 private:
  api::RBounds lane_bounds_;
  api::RBounds driveable_bounds_;
  std::vector<std::unique_ptr<Connection>> connections_;


};



} // namespace biarc
} // namespace maliput





auto b = make_unique<NetworkBuilder>();

b.addLine(xyz(0., 0., 0.),
          0., // heading, radians
          1000., // length, m
          0., // initial elevation gradient, dz/ds
          0., // initial superelevation
          0., // initial superelevation gradient, dTheta/ds

#pragma once

namespace maliput {
namespace biarc {




struct LaneParams {
  double r_min_;
  double r_center_;
  double r_max_;
};


struct Point {
  double x_;
  double y_;
  double heading_; // radians, zero == x direction

  double z_;
  double zdot_;

  double theta_;  // superelevation
  double thetadot_;

  std::vector<LaneParams> lane_config_;
};


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
  Builder builder_;
  std::string id_;
  Point start_;
  Point end_;
  LaneMap lane_map_;
};



class Builder : boost::noncopyable {
 public:
  Builder();

  const RoadGeometry* build() const;

  const Connection* connect(
      const std::string& id,
      const Point& start,
      const Point& end,
      const LaneMap& lane_map);





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

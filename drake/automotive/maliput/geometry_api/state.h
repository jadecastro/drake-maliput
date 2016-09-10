#pragma once

#include <string>

namespace maliput {
namespace geometry_api {


class Lane;


struct Rotation {
  Rotation(double r, double p, double y)
      : roll_(r), pitch_(p), yaw_(y) {}

  double roll_;
  double pitch_;
  double yaw_;
};


struct GeoPosition {
  GeoPosition() {}

  GeoPosition(double x, double y, double z) : x_(x), y_(y), z_(z) {}

  double x_{};
  double y_{};
  double z_{};
};


struct LanePosition {
  LanePosition(double s, double r, double h) : s_(s), r_(r), h_(h) {}

  double s_{};
  double r_{};
  double h_{};
};

struct LaneVelocity {
  double sv_{};
  double rv_{};
  double hv_{};
};

struct LaneAcceleration {
  double sa_{};
  double ra_{};
  double ha_{};
};



struct RoadPosition {
  Lane* lane_;
  LanePosition pos_;
};


struct RBounds {
  double r_min_;
  double r_max_;
};



struct LaneId {
  std::string id_;
};

struct BranchPointId {
  std::string id_;
};

struct SegmentId {
  std::string id_;
};

struct JunctionId {
  std::string id_;
};

struct RoadGeometryId {
  std::string id_;
};



} // namespace geometry_api
} // namespace maliput

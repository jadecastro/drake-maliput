#pragma once

#include <functional>
#include <string>

#include "drake/drakeAutomotive_export.h"

namespace maliput {
namespace geometry_api {


class Lane;


/// The endpoint of a specific Lane.
struct LaneEnd {
  /// Labels for the endpointss of a Lane.  kStart is the "s == 0"
  /// end, and kEnd is the "s == maximum" end.
  enum Which { kStart, kEnd, };

  /// Default constructor.
  LaneEnd() {}

  /// The specified @param end of the specified @param lane.
  LaneEnd(const Lane* lane, Which end) : lane_(lane), end_(end) {}

  const Lane* lane_{};
  Which end_{};

  // NB:  See specialization of std::less<> at end of file, providing
  //      a total ordering used by, e.g., std::map.
};


struct DRAKEAUTOMOTIVE_EXPORT Rotation {
  Rotation(double r, double p, double y)
      : roll_(r), pitch_(p), yaw_(y) {}

  double roll_;
  double pitch_;
  double yaw_;
};


struct DRAKEAUTOMOTIVE_EXPORT GeoPosition {
  GeoPosition() {}

  GeoPosition(double x, double y, double z) : x_(x), y_(y), z_(z) {}

  double x_{};
  double y_{};
  double z_{};
};


struct DRAKEAUTOMOTIVE_EXPORT LanePosition {
  LanePosition(double s, double r, double h) : s_(s), r_(r), h_(h) {}

  double s_{};
  double r_{};
  double h_{};
};

struct DRAKEAUTOMOTIVE_EXPORT LaneVelocity {
  double sv_{};
  double rv_{};
  double hv_{};
};

struct DRAKEAUTOMOTIVE_EXPORT LaneAcceleration {
  double sa_{};
  double ra_{};
  double ha_{};
};



struct DRAKEAUTOMOTIVE_EXPORT RoadPosition {
  Lane* lane_;
  LanePosition pos_;
};


struct DRAKEAUTOMOTIVE_EXPORT RBounds {
  double r_min_;
  double r_max_;
};



struct DRAKEAUTOMOTIVE_EXPORT LaneId {
  std::string id_;
};

struct DRAKEAUTOMOTIVE_EXPORT BranchPointId {
  std::string id_;
};

struct DRAKEAUTOMOTIVE_EXPORT SegmentId {
  std::string id_;
};

struct DRAKEAUTOMOTIVE_EXPORT JunctionId {
  std::string id_;
};

struct DRAKEAUTOMOTIVE_EXPORT RoadGeometryId {
  std::string id_;
};



} // namespace geometry_api
} // namespace maliput


namespace std {

using LaneEnd = maliput::geometry_api::LaneEnd;

template <> struct less<LaneEnd> {
  bool operator()(const LaneEnd& lhs, const LaneEnd& rhs) const {
    return std::less<const void*>()(lhs.lane_, rhs.lane_) ||
        ((lhs.lane_ == rhs.lane_) && (lhs.end_ < rhs.end_));
  }
};
} // namespace std

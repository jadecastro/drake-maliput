#pragma once

#include <functional>
#include <string>

#include "drake/drakeAutomotive_export.h"

namespace maliput {
namespace geometry_api {


class Lane;


/// The endpoint of a specific Lane.
struct LaneEnd {
  /// Labels for the endpoints of a Lane.  kStart is the "s == 0"
  /// end, and kFinish is the other end.
  enum Which { kStart, kFinish, };

  /// An arbitrary strict complete ordering, useful for, e.g., std::map.
  struct StrictOrder {
    bool operator()(const LaneEnd& lhs, const LaneEnd& rhs) const {
      auto as_tuple = [](const LaneEnd& le) {
        return std::tie(le.lane, le.end);
      };
      return as_tuple(lhs) < as_tuple(rhs);
    }
  };


  /// Default constructor.
  LaneEnd() {}

  /// The specified @param end of the specified @param lane.
  LaneEnd(const Lane* _lane, Which _end) : lane(_lane), end(_end) {}

  const Lane* lane{};
  Which end{};
};


struct DRAKEAUTOMOTIVE_EXPORT Rotation {
  Rotation() {}

  Rotation(double _roll, double _pitch, double _yaw)
      : roll(_roll), pitch(_pitch), yaw(_yaw) {}

  double roll{};
  double pitch{};
  double yaw{};
};


struct DRAKEAUTOMOTIVE_EXPORT GeoPosition {
  GeoPosition() {}

  GeoPosition(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

  double x{};
  double y{};
  double z{};
};


struct DRAKEAUTOMOTIVE_EXPORT LanePosition {
  LanePosition() {}

  LanePosition(double _s, double _r, double _h) : s(_s), r(_r), h(_h) {}

  double s{};
  double r{};
  double h{};
};

struct DRAKEAUTOMOTIVE_EXPORT IsoLaneVelocity {
  IsoLaneVelocity() {}

  IsoLaneVelocity(double _sigma_v, double _rho_v, double _eta_v)
      : sigma_v(_sigma_v), rho_v(_rho_v), eta_v(_eta_v) {}

  double sigma_v{};
  double rho_v{};
  double eta_v{};
};


struct DRAKEAUTOMOTIVE_EXPORT RoadPosition {
  RoadPosition() {}

  RoadPosition(const Lane* _lane, const LanePosition& _pos)
      : lane(_lane), pos(_pos) {}

  const Lane* lane{};
  LanePosition pos;
};


struct DRAKEAUTOMOTIVE_EXPORT RBounds {
  double r_min;
  double r_max;
};



struct DRAKEAUTOMOTIVE_EXPORT LaneId {
  std::string id;
};

struct DRAKEAUTOMOTIVE_EXPORT BranchPointId {
  std::string id;
};

struct DRAKEAUTOMOTIVE_EXPORT SegmentId {
  std::string id;
};

struct DRAKEAUTOMOTIVE_EXPORT JunctionId {
  std::string id;
};

struct DRAKEAUTOMOTIVE_EXPORT RoadGeometryId {
  std::string id;
};



}  // namespace geometry_api
}  // namespace maliput

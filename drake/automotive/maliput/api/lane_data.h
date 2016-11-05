#pragma once

#include <functional>
#include <string>

#include "drake/common/drake_export.h"

namespace drake {
namespace maliput {
namespace api {


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


struct DRAKE_EXPORT Rotation {
  Rotation() {}

  Rotation(double _roll, double _pitch, double _yaw)
      : roll(_roll), pitch(_pitch), yaw(_yaw) {}

  double roll{};
  double pitch{};
  double yaw{};
};


struct DRAKE_EXPORT GeoPosition {
  GeoPosition() {}

  GeoPosition(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

  double x{};
  double y{};
  double z{};
};


struct DRAKE_EXPORT LanePosition {
  LanePosition() {}

  LanePosition(double _s, double _r, double _h) : s(_s), r(_r), h(_h) {}

  double s{};
  double r{};
  double h{};
};


struct DRAKE_EXPORT IsoLaneVelocity {
  IsoLaneVelocity() {}

  IsoLaneVelocity(double _sigma_v, double _rho_v, double _eta_v)
      : sigma_v(_sigma_v), rho_v(_rho_v), eta_v(_eta_v) {}

  double sigma_v{};
  double rho_v{};
  double eta_v{};
};


struct DRAKE_EXPORT RoadPosition {
  RoadPosition() {}

  RoadPosition(const Lane* _lane, const LanePosition& _pos)
      : lane(_lane), pos(_pos) {}

  const Lane* lane{};
  LanePosition pos;
};


struct DRAKE_EXPORT RBounds {
  double r_min{};
  double r_max{};
};





}  // namespace geometry_api
}  // namespace maliput
}  // namespace drake

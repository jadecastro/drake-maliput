#pragma once

#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace api {

class CarData {
 public:
  explicit CarData(double& s, double& v, Lane& lane)
      : s_(s), v_(v), lane_(lane) {}

  // Getters
  inline const double& s() const {
    return s_;
  }
  inline const double& v() const {
    return v_;
  }
  inline const Lane& lane() const {
    return lane_;
  }

  // Setters
  inline double& mutable_s() { return s_; }
  inline double& mutable_v() { return v_; }
  inline Lane& mutable_lane() { return lane_; }

  void set(double& s, double& v, Lane& lane) {
    s_ = s;
    v_ = v;
    lane_ = lane;
  }

 private:
  double& s_;
  double& v_;
  Lane& lane_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake

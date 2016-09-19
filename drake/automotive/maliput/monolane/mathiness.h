#pragma once

#include <cmath>

namespace maliput {
namespace monolane {


struct DRAKEAUTOMOTIVE_EXPORT V3 {
  V3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}

  static V3 sum(const V3& a, const V3& b) {
    return V3(a.x + b.x,
              a.y + b.y,
              a.z + b.z);
  }

  double x;
  double y;
  double z;
};

struct DRAKEAUTOMOTIVE_EXPORT V2 {
  V2(double xx, double yy) : x(xx), y(yy) {}

  static V2 midpoint(const V2& a, const V2& b) {
    return V2(0.5 * (a.x + b.x),
              0.5 * (a.y + b.y));
  }

  static V2 difference(const V2& a, const V2& b) {
    return V2(a.x - b.x,
              a.y - b.y);
  }

  double heading() const {
    return std::atan2(y, x);
  }

  double length() const {
    return std::sqrt((x * x) + (y * y));
  }



  double x;
  double y;
};


struct DRAKEAUTOMOTIVE_EXPORT Rot3 {
  Rot3(double yy, double pp, double rr) : yaw(yy), pitch(pp), roll(rr) {}

  V3 apply(const V3& in) const {
    const double sa = std::sin(roll);
    const double ca = std::cos(roll);
    const double sb = std::sin(pitch);
    const double cb = std::cos(pitch);
    const double sg = std::sin(yaw);
    const double cg = std::cos(yaw);

    return V3(
        ((cb * cg) * in.x) +
        ((-ca*sg + sa*sb*cg) * in.y) +
        ((-sa*sg - ca*sb*cg) * in.z),

        ((cb*sg) * in.x) +
        ((ca*cg + sa*sb*sg) * in.y) +
        ((sa*cg - ca*sb*sg) * in.z),

        ((sb) * in.x) +
        ((-sa*cb) * in.y) +
        ((ca*cb) * in.z));
  }

  double yaw;
  double pitch;
  double roll;
};


// parameterized on domain p in [0, 1]
class DRAKEAUTOMOTIVE_EXPORT CubicPolynomial {
 public:
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    // TODO(maddog)  For now, completely bogus linear distance
    //               from (0,0) to (1,df):
    s_1_ = std::sqrt(1. + (df * df));
  }

  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  double fdot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }


  // TODO(maddog)  Perform a proper arc-length parameterization!
  double s_p(double p) const {
    return s_1_ * p;
  }

  double p_s(double s) const {
    return s / s_1_;
  }

  double f_s(double s) const {
    return f_p(p_s(s));
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};

}  // namespace monolane
}  // namespace maliput

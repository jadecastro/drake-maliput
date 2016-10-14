#include "lane.h"

#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include "drake/common/drake_assert.h"

namespace maliput {
namespace monolane {

namespace api = maliput::geometry_api;


const api::Segment* Lane::do_segment() const { return segment_; }

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: { return start_bp_; }
    case api::LaneEnd::kEnd:   { return end_bp_; }
  }
  DRAKE_ABORT();
}

const api::SetOfLaneEnds* Lane::DoGetBranches(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetBranches({this, which_end});
}

const boost::optional<api::LaneEnd>& Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}


double Lane::do_length() const {
  return elevation().s_p(1.0) * p_scale_;
}


Rot3 Lane::rot3_of_p_(const double p) const {
  return Rot3(heading_of_p_(p),
              -std::atan(elevation().fdot_p(p)),
              superelevation().f_p(p) * p_scale_);
}

double Lane::p_from_s_(const double s) const {
  return elevation().p_s(s / p_scale_);
}


V3 Lane::W_prime_of_prh_(const double p, const double r, const double h,
                         const Rot3& gba) const {
  const V2 G_prime = xy_dot_of_p_(p);
  // TODO(maddog)  Until s(p) is integrated properly.
  const double g_prime = elevation().fdot_p(p);
  //const double g_prime = elevation().fake_gprime(p);

  const Rot3& R = gba;
  const double alpha = R.roll;
  const double beta = R.pitch;
  const double gamma = R.yaw;

  const double ca = std::cos(alpha);
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // TODO(maddog)  Hmm... is d_alpha scaled correctly?
  const double d_alpha = superelevation().fdot_p(p) * p_scale_;
  const double d_beta = -cb * cb * elevation().fddot_p(p);
  const double d_gamma = heading_dot_of_p_(p);

  // TODO(maddog)  NEEDS ALPHA SIGN CORRECTION.
  return
      V3(G_prime.x,
         G_prime.y,
         p_scale_ * g_prime) +

      V3((((sa*sg)+(ca*sb*cg))*r + ((ca*sg)-(sa*sb*cg))*h),
         (((-sa*cg)+(ca*sb*sg))*r - ((ca*cg)+(sa*sb*sg))*h),
         ((ca*cb)*r + (-sa*cb)*h))
      * d_alpha +

      V3(((sa*cb*cg)*r + (ca*cb*cg)*h),
         ((sa*cb*sg)*r + (ca*cb*sg)*h),
         ((-sa*sb)*r - (ca*sb)*h))
      * d_beta +

      V3((((-ca*cg)-(sa*sb*sg))*r + ((+sa*cg)-(ca*sb*sg))*h),
         (((-ca*sg)+(sa*sb*cg))*r + ((sa*sg)+(ca*sb*cg))*h),
         0)
      * d_gamma;
}


V3 Lane::s_hat_of_prh_(const double p, const double r, const double h,
                       const Rot3& gba) const {
  const V3 W_prime = W_prime_of_prh_(p, r, h, gba);
  return W_prime * (1.0 / W_prime.magnitude());
}


V3 Lane::r_hat_of_gba_(const Rot3& gba) const {
  return gba.apply({0., 1., 0.});
}


api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  // Recover parameter p from arc-length position s.
  const double p = p_from_s_(lane_pos.s_);
  // Calculate z (elevation) of (s,0,0);
  const double z = elevation().f_p(p) * p_scale_;
  // Calculate x,y of (s,0,0).
  const V2 xy = xy_of_p_(p);
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 ypr = rot3_of_p_(p);

  // Rotate (0,r,h) and sum with mapped (s,0,0).
  const V3 xyz = V3::sum(ypr.apply({0., lane_pos.r_, lane_pos.h_}),
                         {xy.x, xy.y, z});
  return {xyz.x, xyz.y, xyz.z};
}


api::Rotation Lane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  // Recover linear parameter p from arc-length position s.
  const double p = p_from_s_(lane_pos.s_);
  const double r = lane_pos.r_;
  const double h = lane_pos.h_;
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 gba = rot3_of_p_(p);

  // Calculate s,r basis vectors at (s,r,h)...
  const V3 s_hat = s_hat_of_prh_(p, r, h, gba);
  const V3 r_hat = r_hat_of_gba_(gba);
  // ...and then derive orientation from those basis vectors.
  const double gamma = std::atan2(s_hat.y,
                                  s_hat.x);
  const double beta = std::atan2(-s_hat.z,
                                 V2(s_hat.x, s_hat.y).length());
  const double cb = std::cos(beta);
  const double alpha =
      std::atan2(r_hat.z / cb,
                 ((r_hat.y * s_hat.x) - (r_hat.x * s_hat.y)) / cb);
  return api::Rotation(alpha, beta, gamma);
}


void Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity,
    api::LanePosition* position_dot) const {

  const double p = p_from_s_(position.s_);
  const double r = position.r_;
  const double h = position.h_;

  // TODO(maddog)  Until s(p) is integrated properly.
  // const double g_prime = elevation().fdot_p(p);
  const double g_prime = elevation().fake_gprime(p);

  const Rot3 R = rot3_of_p_(p);
  const V3 W_prime = W_prime_of_prh_(p, r, h, R);

  const double d_s = p_scale_ * std::sqrt(1 + (g_prime * g_prime));

  *position_dot = {d_s / W_prime.magnitude() * velocity.sigma_v_,
                   velocity.rho_v_,
                   velocity.eta_v_};
}




}  // namespace monolane
}  // namespace maliput

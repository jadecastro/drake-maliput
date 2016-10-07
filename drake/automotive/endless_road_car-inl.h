#pragma once

/// @file
/// Template method implementations for endless_road_car.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "endless_road_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/geometry_api/state.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadCar<T>::EndlessRoadCar(
    const maliput::utility::InfiniteCircuitRoad* road,
    const double s0, const double r0, const double speed)
    : road_(road), speed_(speed) {
//LATER  this->DeclareInputPort(systems::kVectorValued,
//LATER                         DrivingCommandIndices::kNumCoordinates,
//LATER                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
bool EndlessRoadCar<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void EndlessRoadCar<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state()->get_state();
  const EndlessRoadCarState<T>* const state =
      dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the output pointer.
  EndlessRoadCarState<T>* const output_vector =
      dynamic_cast<EndlessRoadCarState<T>*>(output->GetMutableVectorData(0));
  DRAKE_ASSERT(output_vector);

  DoEvalOutput(*state, output_vector);
}

template <typename T>
void EndlessRoadCar<T>::DoEvalOutput(const EndlessRoadCarState<T>& state,
                                     EndlessRoadCarState<T>* output) const {
  output->set_value(state.get_value());
}

template <typename T>
void EndlessRoadCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state()->get_state();
  const EndlessRoadCarState<T>* const state =
      dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

//LATER  // Obtain the input.
//LATER  const systems::VectorBase<T>* const vector_input =
//LATER      this->EvalVectorInput(context, 0);
//LATER  DRAKE_ASSERT(vector_input);
//LATER  const DrivingCommand<T>* const input =
//LATER      dynamic_cast<const DrivingCommand<T>*>(vector_input);
//LATER  DRAKE_ASSERT(input);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_state();
  DRAKE_ASSERT(vector_derivatives);
  EndlessRoadCarState<T>* const rates =
      dynamic_cast<EndlessRoadCarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates);

//LATER  DoEvalTimeDerivatives(*state, *input, rates);
  DoEvalTimeDerivatives(*state, rates);
}

template <typename T>
void EndlessRoadCar<T>::DoEvalTimeDerivatives(
    const EndlessRoadCarState<T>& state,
//LATER    const DrivingCommand<T>& input,
    EndlessRoadCarState<T>* rates) const {

  maliput::geometry_api::LanePosition lane_position(state.s(), state.r(), 0.);
  maliput::geometry_api::IsoLaneVelocity lane_velocity(
      state.sigma_dot(), state.rho_dot(), 0.);
  maliput::geometry_api::LanePosition derivatives;
  road_->lane()->EvalMotionDerivatives(
      lane_position, lane_velocity, &derivatives);

  rates->set_s(derivatives.s_);
  rates->set_r(derivatives.r_);
  // Ignore derivatives.h_, which should be zero anyhow.

  // No inputs yet:  zero accelerations.
  const double sigma_ddot = 0.;
  const double rho_ddot = 0.;
  rates->set_sigma_dot(sigma_ddot);
  rates->set_rho_dot(rho_ddot);

#if 0
  // Apply simplistic throttle.
  T new_velocity =
      state.velocity() + (input.throttle() * config_.max_acceleration() *
                          config_.velocity_lookahead_time());
  new_velocity = std::min(new_velocity, config_.max_velocity());

  // Apply simplistic brake.
  new_velocity += input.brake() * -config_.max_acceleration() *
                  config_.velocity_lookahead_time();
  new_velocity = std::max(new_velocity, static_cast<T>(0.));

  // Apply steering.
  T sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle, config_.max_abs_steering_angle());
  sane_steering_angle = std::max(
      sane_steering_angle, static_cast<T>(-config_.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config_.wheelbase();

  rates->set_x(state.velocity() * cos(state.heading()));
  rates->set_y(state.velocity() * sin(state.heading()));
  rates->set_heading(curvature * state.velocity());
  rates->set_velocity((new_velocity - state.velocity()) *
                      config_.velocity_kp());
#endif
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
EndlessRoadCar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<EndlessRoadCarState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> EndlessRoadCar<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadCarState<T>>();
}

}  // namespace automotive
}  // namespace drake

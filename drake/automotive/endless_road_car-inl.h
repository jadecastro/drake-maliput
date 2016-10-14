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
    bool ignore_input_jackass,
    const SimpleCarConfig<T>& config)
    : road_(road), config_(config), ignore_input_jackass_(ignore_input_jackass) {
  this->DeclareInputPort(systems::kVectorValued,
                         DrivingCommandIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}


template <typename T>
SimpleCarConfig<T> EndlessRoadCar<T>::get_default_config() {
  constexpr double kInchToMeter = 0.0254;
  constexpr double kDegToRadian = 0.0174532925199;
  // This approximates a 2010 Toyota Prius.
  SimpleCarConfig<T> result;
  result.set_wheelbase(static_cast<T>(106.3 * kInchToMeter));
  result.set_track(static_cast<T>(59.9 * kInchToMeter));
  result.set_max_abs_steering_angle(static_cast<T>(27 * kDegToRadian));
  result.set_max_velocity(static_cast<T>(45.0));  // meters/second
  result.set_max_acceleration(static_cast<T>(4.0));  // meters/second**2
  result.set_velocity_lookahead_time(static_cast<T>(1.0));  // second
  result.set_velocity_kp(static_cast<T>(1.0));  // Hz
  return result;
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

  // Obtain the input.
  const systems::VectorBase<T>* const vector_input =
      this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(vector_input);
  const DrivingCommand<T>* const input =
      dynamic_cast<const DrivingCommand<T>*>(vector_input);
  if (!ignore_input_jackass_) {
  DRAKE_ASSERT(input);
  }

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_state();
  DRAKE_ASSERT(vector_derivatives);
  EndlessRoadCarState<T>* const rates =
      dynamic_cast<EndlessRoadCarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates);

  if (ignore_input_jackass_) {
    DrivingCommand<T> zero_input;
    DoEvalTimeDerivatives(*state, zero_input, rates);
  } else {
    DoEvalTimeDerivatives(*state, *input, rates);
  }
}


template <typename T>
void EndlessRoadCar<T>::DoEvalTimeDerivatives(
    const EndlessRoadCarState<T>& state,
    const DrivingCommand<T>& input,
    EndlessRoadCarState<T>* rates) const {

  // Position + velocity ---> position derivatives.
  maliput::geometry_api::LanePosition lane_position(state.s(), state.r(), 0.);
  maliput::geometry_api::IsoLaneVelocity lane_velocity(
      state.sigma_dot(), state.rho_dot(), 0.);
  maliput::geometry_api::LanePosition derivatives;
  road_->lane()->EvalMotionDerivatives(
      lane_position, lane_velocity, &derivatives);

  rates->set_s(derivatives.s);
  rates->set_r(derivatives.r);
  // Ignore derivatives.h_, which should be zero anyhow.

  // Velocity + control inputs ---> velocity derivatives.
  const T speed = std::sqrt((state.sigma_dot() * state.sigma_dot()) +
                            (state.rho_dot() * state.rho_dot()));
  // "heading angle w.r.t. s_hat == zero angle"
  const T heading = std::atan2(state.rho_dot(), state.sigma_dot());

  // Simplistic throttle and brake --> longitudinal acceleration.
  const T forward_acceleration = (input.throttle() - input.brake())
      * config_.max_acceleration();

  // Simplistic steering --> lateral acceleration == centripetal acceleration.
  T sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle, config_.max_abs_steering_angle());
  sane_steering_angle = std::max(
      sane_steering_angle, static_cast<T>(-config_.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config_.wheelbase();
  const T lateral_acceleration = speed * speed * curvature;

  const double sigma_ddot =
      (forward_acceleration * std::cos(heading)) -
      (lateral_acceleration * std::sin(heading));
  const double rho_ddot =
      (forward_acceleration * std::sin(heading)) +
      (lateral_acceleration * std::cos(heading));
  rates->set_sigma_dot(sigma_ddot);
  rates->set_rho_dot(rho_ddot);
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

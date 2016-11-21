#include "drake/automotive/endless_road_simple_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/vector_base.h"

// This is used indirectly to allow DRAKE_ASSERT on symbolic::Expression.
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadSimpleCar<T>::EndlessRoadSimpleCar(
    const maliput::utility::InfiniteCircuitRoad* road)
    : road_(road) {
  this->DeclareInputPort(systems::kVectorValued,
                         2,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
bool EndlessRoadSimpleCar<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void EndlessRoadSimpleCar<T>::EvalOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
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
void EndlessRoadSimpleCar<T>::DoEvalOutput(const EndlessRoadCarState<T>& state,
                                EndlessRoadCarState<T>* output) const {
  output->set_value(state.get_value());
}

template <typename T>
void EndlessRoadSimpleCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const EndlessRoadCarState<T>* const state =
      dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the input acceleration requests to the car.
  // TODO(jadecastro): are acceleration components the best input?
  const systems::VectorBase<T>* const vector_input =
      this->EvalVectorInput(context, 0);
  const Accelerations accelerations = [&](){
    return Accelerations (vector_input->GetAtIndex(0),
                          vector_input->GetAtIndex(1));
  }();

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(vector_derivatives);
  EndlessRoadCarState<T>* const rates =
      dynamic_cast<EndlessRoadCarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates);

  DoEvalTimeDerivatives(*state, accelerations, rates);
}

template <typename T>
void EndlessRoadSimpleCar<T>::DoEvalTimeDerivatives(
    const EndlessRoadCarState<T>& state,
    const Accelerations& accelerations,
    EndlessRoadCarState<T>* rates) const {

  // Position + velocity ---> position derivatives.
  maliput::api::LanePosition lane_position(state.s(), state.r(), 0.);
  maliput::api::IsoLaneVelocity lane_velocity(
      state.speed() * std::cos(state.heading()),
      state.speed() * std::sin(state.heading()),
      0.);
  maliput::api::LanePosition derivatives;
  road_->lane()->EvalMotionDerivatives(
      lane_position, lane_velocity, &derivatives);

  rates->set_s(derivatives.s);
  rates->set_r(derivatives.r);
  // Ignore derivatives.h_, which should be zero anyhow.

  const double speed_dot = accelerations.forward;
  const double heading_dot =
      (state.speed() == 0.) ? 0. : (accelerations.lateral / state.speed());

  rates->set_speed(speed_dot);
  rates->set_heading(heading_dot);
  // TODO(jadecastro): Similar to SimpleCar, cap the steering angle
  // and max/min accelerations based on vehicle-intrinsic params.
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
EndlessRoadSimpleCar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<EndlessRoadCarState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
EndlessRoadSimpleCar<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadCarState<T>>();
}

// These instantiations must match the API documentation in simple_car.h.
// TODO(jadecastro): Fix to allow for the wider set of scalar types.
template class EndlessRoadSimpleCar<double>;
//template class EndlessRoadSimpleCar<drake::AutoDiffXd>;
//template class EndlessRoadSimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake

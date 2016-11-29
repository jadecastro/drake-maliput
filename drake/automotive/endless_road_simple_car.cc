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
                const maliput::utility::InfiniteCircuitRoad* road,
                const T& s_init, const T& r_init,
                const T& v_init, const T& heading_init)
  : road_(road),
    s_init_(s_init), r_init_(r_init),
    v_init_(v_init), heading_init_(heading_init) {
  std::cerr << "   EndlessRoadSimpleCar s_init: " << s_init_ << " \n";
  this->DeclareInputPort(systems::kVectorValued,
                         2,
                         systems::kContinuousSampling);
  // Declare an output port collecting the states.
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
  // Declare an output port collecting the s-axis position and velocity.
  this->DeclareOutputPort(systems::kVectorValued,
                          2,
                          systems::kContinuousSampling);
  this->DeclareContinuousState(EndlessRoadCarStateIndices::kNumCoordinates);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
EndlessRoadSimpleCar<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
EndlessRoadSimpleCar<T>::get_s_axis_output_port() const {
  return systems::System<T>::get_output_port(1);
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

  std::cerr << " %%%%%%%%%% EndlessRoadSimpleCar::EvalOutput ..." <<
      std::endl;

  std::cerr << "EndlessRoadSimpleCar::EvalOutput...\n";
  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  //const EndlessRoadCarState<T>* const state =
  //    dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  //DRAKE_ASSERT(state);
  std::cerr << "EndlessRoadSimpleCar::EvalOutput 1...\n";

  // Obtain the output pointer for the output port containing the state vector.
  //EndlessRoadCarState<T>* const output_vector_state =
  //    dynamic_cast<EndlessRoadCarState<T>*>(output->GetMutableVectorData(0));
  systems::BasicVector<T>*
    output_vector_state = output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector_state);
  std::cerr << "EndlessRoadSimpleCar::EvalOutput 2...\n";

  std::cerr << "  s: " << context_state.GetAtIndex(0) << "\n";
  std::cerr << "  r: " << context_state.GetAtIndex(1) << "\n";
  std::cerr << "  heading: " << context_state.GetAtIndex(2) << "\n";
  std::cerr << "  speed: " << context_state.GetAtIndex(3) << "\n";

  output_vector_state->SetAtIndex(0, context_state.GetAtIndex(0));
  output_vector_state->SetAtIndex(1, context_state.GetAtIndex(1));
  output_vector_state->SetAtIndex(2, context_state.GetAtIndex(2));
  output_vector_state->SetAtIndex(3, context_state.GetAtIndex(3));

  //output_vector_state->set_value(state->get_value());

  std::cerr << "EndlessRoadSimpleCar::EvalOutput 3...\n";
  // Set the output port collecting the s-axis quantities.
  systems::BasicVector<T>*
    output_vector_s_axis = output->GetMutableVectorData(1);
  output_vector_s_axis->SetAtIndex(0, context_state.GetAtIndex(0));
  output_vector_s_axis->SetAtIndex(1, context_state.GetAtIndex(3));

  std::cerr << "EndlessRoadSimpleCar::EvalOutput.\n";
}

template <typename T>
void EndlessRoadSimpleCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  std::cerr << " ****** EndlessRoadSimpleCar::EvalTimeDerivatives ..." <<
      std::endl;

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  //const EndlessRoadCarState<T>* const state =
  //    dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  //DRAKE_ASSERT(state);

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
  //EndlessRoadCarState<T>* const rates =
  //    dynamic_cast<EndlessRoadCarState<T>*>(vector_derivatives);
  //DRAKE_ASSERT(rates);

  DoEvalTimeDerivatives(context_state, accelerations, vector_derivatives);
}

template <typename T>
void EndlessRoadSimpleCar<T>::DoEvalTimeDerivatives(
    const systems::VectorBase<T>& state,
    const Accelerations& accelerations,
    systems::VectorBase<T>* rates) const {

  std::cerr << "   ****  DoEvalTimeDerivatives s_init: " <<
      state.GetAtIndex(0) << std::endl;

  // Position + velocity ---> position derivatives.
  maliput::api::LanePosition
    lane_position(state.GetAtIndex(0), state.GetAtIndex(1), 0.);
  maliput::api::IsoLaneVelocity lane_velocity(
      state.GetAtIndex(3) * std::cos(state.GetAtIndex(2)),
      state.GetAtIndex(3) * std::sin(state.GetAtIndex(2)),
      0.);
  maliput::api::LanePosition derivatives;
  road_->lane()->EvalMotionDerivatives(
      lane_position, lane_velocity, &derivatives);

  rates->SetAtIndex(0, derivatives.s);
  rates->SetAtIndex(1, derivatives.r);
  // Ignore derivatives.h_, which should be zero anyhow.

  const double speed_dot = accelerations.forward;
  const double heading_dot =
      (state.GetAtIndex(3) == 0.) ? 0. :
    (accelerations.lateral / state.GetAtIndex(3));

  rates->SetAtIndex(3, speed_dot);
  rates->SetAtIndex(2, heading_dot);
  // TODO(jadecastro): Similar to SimpleCar, cap the steering angle
  // and max/min accelerations based on vehicle-intrinsic params.
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
void EndlessRoadSimpleCar<T>::SetDefaultState(systems::Context<T>* context)
    const {
  // Obtain mutable references to the contexts for each car.
  DRAKE_DEMAND(context != nullptr);
  systems::ContinuousState<T>* state = context->get_mutable_continuous_state();
  DRAKE_DEMAND(state != nullptr);

  std::cerr << "  s_init: " << s_init_ << "\n";

  // Set the elements of the state vector to pre-defined values.
  (*state->get_mutable_vector())[0] = s_init_;  // initial s
  (*state->get_mutable_vector())[1] = r_init_;  // initial r
  (*state->get_mutable_vector())[2] = v_init_;  // initial v
  (*state->get_mutable_vector())[3] = heading_init_;  // initial heading
}

  /*
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
  */

// These instantiations must match the API documentation in simple_car.h.
// TODO(jadecastro): Fix to allow for the wider set of scalar types.
template class EndlessRoadSimpleCar<double>;
//template class EndlessRoadSimpleCar<drake::AutoDiffXd>;
//template class EndlessRoadSimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake

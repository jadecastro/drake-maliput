#pragma once

/// @file
/// Template method implementations for endless_road_oracle.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "endless_road_oracle.h"

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
EndlessRoadOracle<T>::EndlessRoadOracle(
    const maliput::utility::InfiniteCircuitRoad* road,
    const int num_cars)
    : road_(road), num_cars_(num_cars) {
  // Declare an input and output for each car.
  for (int i = 0; i < num_cars; ++i) {
    inports_.push_back(
        this->DeclareInputPort(systems::kVectorValued,
                               EndlessRoadCarStateIndices::kNumCoordinates,
                               systems::kContinuousSampling));
    outports_.push_back(
        this->DeclareOutputPort(systems::kVectorValued,
                                EndlessRoadOracleOutputIndices::kNumCoordinates,
                                systems::kContinuousSampling));
  }
}


template <typename T>
void EndlessRoadOracle<T>::EvalOutput(const systems::Context<T>& context,
                                      systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the inputs.
  std::vector<const EndlessRoadCarState<T>*> car_inputs;
  for (int i = 0; i < num_cars_; ++i) {
    const systems::VectorBase<T>* const vector_input =
        this->EvalVectorInput(context, i);
    DRAKE_ASSERT(vector_input);
    const EndlessRoadCarState<T>* const car_input =
        dynamic_cast<const EndlessRoadCarState<T>*>(vector_input);
    DRAKE_ASSERT(car_input);
    car_inputs.push_back(car_input);
  }

  // Obtain the output pointers.
  std::vector<EndlessRoadOracleOutput<T>*> oracle_outputs;
  for (int i = 0; i < num_cars_; ++i) {
    EndlessRoadOracleOutput<T>* const output_vector =
        dynamic_cast<EndlessRoadOracleOutput<T>*>(
            output->GetMutableVectorData(i));
    DRAKE_ASSERT(output_vector);
    oracle_outputs.push_back(output_vector);
  }

  DoEvalOutput(car_inputs, oracle_outputs);
}


template <typename T>
void EndlessRoadOracle<T>::DoEvalOutput(
    const std::vector<const EndlessRoadCarState<T>*>& car_inputs,
    std::vector<EndlessRoadOracleOutput<T>*>& oracle_outputs) const {

  // Calculate longitudinal position of each car in the circuit.
  // Sort cars by longitudinal position.
  std::multimap<double, int> sorted_by_s;
  for (int i = 0; i < num_cars_; ++i) {
    const double circuit_s = std::fmod(car_inputs[i]->s(),
                                       road_->cycle_length());
    sorted_by_s.emplace(circuit_s, i);
  }
  // For each car:
  //  - find nearest other car that is:
  //     a) ahead of self, and
  //     b) has a lateral position within XXXXXX bounds of self.
  //  - compute/record:
  //     * net true longitudinal distance to other car (delta sigma)
  //     * true longitudinal velocity difference (delta sigma-dot)
  for (int i = 0; i < num_cars_; ++i) {
    const EndlessRoadCarState<T>* self = car_inputs[i];
    const double self_circuit_s = std::fmod(self->s(),
                                            road_->cycle_length());

    auto it = sorted_by_s.find(self_circuit_s);
    DRAKE_DEMAND(it != sorted_by_s.end());

    while (it->first <= self_circuit_s) {
      ++it;
      if (it == sorted_by_s.cend()) {
        it = sorted_by_s.begin();
        break;
      }
    }
    const EndlessRoadCarState<T>* other = car_inputs[it->second];
    const double other_circuit_s = std::fmod(other->s(),
                                             road_->cycle_length());

    const double kCarLength = 2.;  // TODO(maddog) Get from somewhere else.
    // TODO(maddog) Do a correct distance measurement (not just delta-s).
    const double net_delta_sigma =
        ((other_circuit_s >= self_circuit_s)
         ? (other_circuit_s - self_circuit_s)
         : (other_circuit_s + road_->cycle_length() - self_circuit_s))
        - kCarLength;

    oracle_outputs[i]->set_net_delta_sigma(net_delta_sigma);
    oracle_outputs[i]->set_delta_sigma_dot(
        self->sigma_dot() - other->sigma_dot());
  }
}


template <typename T>
std::unique_ptr<systems::BasicVector<T>> EndlessRoadOracle<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadOracleOutput<T>>();
}

}  // namespace automotive
}  // namespace drake

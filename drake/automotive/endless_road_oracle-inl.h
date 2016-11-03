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
  // The goal here is, for each car, to calculate the distance and
  // differential velocity to the nearest other car/obstacle ahead.
  // We consider a couple of different varieties of "car/obstacle ahead":
  //  a) cars ahead on the InfiniteCircuitRoad circuit, which covers
  //      normal car-following behavior;
  //  b) cars approaching from the right, on intersecting lanes in
  //      the base RoadGeometry --- this hopefully produces some non-crashing
  //      behavior at intersections;
  // TODO(maddog)  Actually do part (c).
  //  c) cars in a merging lane to the left --- this hopefully produces
  //      so non-crashing merging behaviors.

  std::vector<boost::optional<T>> collisions_by_car =
      AssessIntersections(car_inputs);

  // Calculate longitudinal position of each car in the circuit.
  // Sort cars by longitudinal position:  use an ordered multimap that
  // maps s-position --> car-index.
  std::multimap<double, int> sorted_by_s;
  for (int i = 0; i < num_cars_; ++i) {
    const double circuit_s = road_->lane()->circuit_s(car_inputs[i]->s());
    sorted_by_s.emplace(circuit_s, i);
  }
  // For each car:
  //  - find nearest other car that is:
  //     a) ahead of self, and
  //     b) has a lateral position within XXXXXX bounds of self.
  //  - compute/record:
  //     * net true longitudinal distance to other car (delta sigma)
  //     * true longitudinal velocity difference (delta sigma-dot)
  const double kCarLength = 4.6;  // TODO(maddog) Get from somewhere else.
  // TODO(maddog)  Do something reasonable if num_cars_ < 2.
  for (int i = 0; i < num_cars_; ++i) {
    const EndlessRoadCarState<T>* self = car_inputs[i];
    const double self_circuit_s = road_->lane()->circuit_s(self->s());

//DBG    if (sorted_by_s.find(self_circuit_s) == sorted_by_s.end()) {
//DBG      std::cerr << "ARGH? "
//DBG                << "  i " << i
//DBG                << "  s " << self->s()
//DBG                << "  cs " << self_circuit_s
//DBG                << std::endl;
//DBG    }
    DRAKE_DEMAND(sorted_by_s.find(self_circuit_s) != sorted_by_s.end());

    // Find the next car which is ahead of self_circuit_s, by at least
    // a car-length.
    auto other_it = sorted_by_s.upper_bound(self_circuit_s + kCarLength);
    if (other_it == sorted_by_s.end()) {
      other_it = sorted_by_s.upper_bound(
          self_circuit_s + kCarLength - road_->cycle_length());
      DRAKE_DEMAND(other_it != sorted_by_s.end());
    }
    const EndlessRoadCarState<T>* other = car_inputs[other_it->second];
    const double other_circuit_s = road_->lane()->circuit_s(other->s());
    DRAKE_DEMAND(other_circuit_s == other_it->first);

    // TODO(maddog) Do a correct distance measurement (not just delta-s).
    const double net_delta_sigma =
        ((other_circuit_s >= self_circuit_s)
         ? (other_circuit_s - self_circuit_s)
         : (other_circuit_s + road_->cycle_length() - self_circuit_s))
        - kCarLength;
    DRAKE_DEMAND(net_delta_sigma > 0.);

    oracle_outputs[i]->set_net_delta_sigma(net_delta_sigma);
    oracle_outputs[i]->set_delta_sigma_dot(
        (self->speed() * std::cos(self->heading())) -
        (other->speed() * std::cos(other->heading())));

//DBG    if (i == -1) {
//DBG      std::cerr << "Hmm? "
//DBG                << "  i " << i
//DBG                << "  s " << self->s()
//DBG                << "  cs " << self_circuit_s
//DBG                << "  other " << other_it->second
//DBG                << "  os " << other->s()
//DBG                << "  ocs " << other_circuit_s
//DBG                << "  nds " << net_delta_sigma
//DBG                << std::endl;
//DBG    }
    if ((collisions_by_car[i]) && (*collisions_by_car[i] < net_delta_sigma)) {
//DBG      std::cerr << "COLL: " << i
//DBG                << "  at dist " << *collisions_by_car[i]
//DBG                << "  not " << net_delta_sigma
//DBG                << std::endl;
      oracle_outputs[i]->set_net_delta_sigma(*collisions_by_car[i]);
      oracle_outputs[i]->set_delta_sigma_dot(
          (self->speed() * std::cos(self->heading())) - 0.);
    }
  }
}


template <typename T>
std::vector<boost::optional<T>> EndlessRoadOracle<T>::AssessIntersections(
    const std::vector<const EndlessRoadCarState<T>*>& car_inputs) const {

  // Time to look-ahead for intersections.
  // TODO(maddog) Should be a multiple of IDM headway h.
#if 1
  const T kEventHorizon{2.0};  // seconds
  // Indexing Phase.
  // For each car, measure/record the 'time-in' and 'time-out' to all junctions
  // expected to be traversed within kEventHorizon.
  struct TimeBox {
    int car_index;
    const maliput::geometry_api::Lane* lane;
    T time_in;
    T time_out;
    T s_in;
    T s_out;
  };
  std::map<const maliput::geometry_api::Junction*,
           std::vector<TimeBox>> boxes_by_junction;
  std::map<int,
           std::vector<const maliput::geometry_api::Junction*>> junctions_by_car;

  for (int i = 0; i < num_cars_; ++i) {
    const EndlessRoadCarState<T>* state = car_inputs[i];
    const double sigma_dot =  state->speed() * std::cos(state->heading());
    const T lookahead_distance = sigma_dot * kEventHorizon;
    DRAKE_DEMAND(lookahead_distance < (0.5 * road_->cycle_length()));
    const T s0 = road_->lane()->circuit_s(state->s());
    const T lookahead_horizon = s0 + lookahead_distance;
    // TODO(maddog)  This does not handle s_dot < 0 correctly.

    int path_index = road_->GetPathIndex(s0);
    maliput::utility::InfiniteCircuitRoad::Record path_record =
        road_->path_record(path_index);
    double s_in = s0; // ...because we start somewhere in the middle.

    while (s_in < lookahead_horizon) {
      double s_out = path_record.end_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (s_out < s0) { s_out += road_->cycle_length(); }

      // TODO(maddog) time in/out should account for length of vehicle, too.
      const T time_in = (s_in - s0) / sigma_dot;
      const T time_out = (s_out - s0) / sigma_dot;
      const TimeBox box = TimeBox{i, path_record.lane,
                                  time_in, time_out,
                                  s_in, s_out};
      const maliput::geometry_api::Junction* junction =
          path_record.lane->segment()->junction();
      boxes_by_junction[junction].push_back(box);
      junctions_by_car[i].push_back(junction);
//DBG      if ((junction->id().id == "j:A12-YR1") ||
//DBG          (junction->id().id == "j:B1R-X12")) {
//DBG        std::cerr << "car " << i
//DBG                  << "  " << junctions_by_car[i].size() << " jnxs"
//DBG                  << "   At " << junction->id().id
//DBG                  << "  " << path_record.lane->id().id
//DBG                  << "  [" << box.time_in << ", " << box.time_out << "]"
//DBG                  << std::endl;
//DBG      }
      // TODO(maddog) Index should decrement for s_dot < 0.
      if (++path_index >= road_->num_path_records()) {
        path_index = 0;
      }
      path_record = road_->path_record(path_index);
      s_in = path_record.start_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (s_in < s0) { s_in += road_->cycle_length(); }
    }
  }

  // Measure Phase.
  // For each car, find cars which are entering the same junction at the
  // same time as this car.  We only want to pay attention to cars coming
  // from the right, and ultimately only care about the nearest such car.
#endif
#if 1
  std::vector<boost::optional<T>> collision_points_by_car;
  for (int i = 0; i < num_cars_; ++i) {
    const EndlessRoadCarState<T>* self_state = car_inputs[i];
    const T s0 = road_->lane()->circuit_s(self_state->s());

    boost::optional<T> might_collide_at = boost::none;

    // Iterate over the junctions which car 'i' participates in.
    for (const auto& junction : junctions_by_car[i]) {

      // Find the TimeBox for this car.
      const TimeBox self_box = [&](){
        // TODO(maddog)  Linear search is dumb.
        for (const auto& box : boxes_by_junction[junction]) {
          if (box.car_index == i) { return box; }
        }
        DRAKE_ABORT();
      }();
      DRAKE_DEMAND(self_box.time_in < self_box.time_out);

      // Are there any intersecting TimeBoxes from other cars?
      const bool might_collide = [&](){
        // If self is *already in* the intersection, don't worry about it.
        if (self_box.time_in == 0.) { return false; }
        for (const auto& other_box : boxes_by_junction[junction]) {
          // Skip self (car 'i').
          if (other_box.car_index == i) { continue; }
          // Skip same lane (which cannot be an intersecting path).
          if (other_box.lane == self_box.lane) { continue;}
          // Skip other if it enters after self enters (self is first),
          // or exits before self enters (other is out of the way).

//DBG          std::cerr << "MC  " << i << " vs " << other_box.car_index
//DBG          << ": s[" << self_box.time_in << ", " << self_box.time_out
//DBG          << "  o[" << other_box.time_in << ", " << other_box.time_out
//DBG          << std::endl;

          DRAKE_DEMAND(other_box.time_in < other_box.time_out);
          if ((other_box.time_in > self_box.time_in) ||
              (other_box.time_out < self_box.time_in)) { continue; }

          // TODO(maddog) Compare speeds?  We really want the slower car
          // to go slower, and faster faster, right?

          // TODO(maddog) Skip cars coming from left.  (necessary?)
//DBG          std::cerr << "MIGHT COLLIDE!" << std::endl;
          return true;
        }
        return false;
      }();

      if (might_collide) {
        if (might_collide_at) {
          might_collide_at = std::min(*might_collide_at, self_box.s_in);
        } else {
          might_collide_at = self_box.s_in;
        }
      }

    }
    if (might_collide_at) {
    }
    if (might_collide_at) { //// &&
        // TODO(maddog)  Yeah... why are we getting zero diffs?
        //////(*might_collide_at > car_inputs[i]->s())) {
//DBG      std::cerr << "car " << i << " might collide in "
//DBG                << *might_collide_at
//DBG                << "  net-dist " << (*might_collide_at - s0)
//DBG                << std::endl;
      collision_points_by_car.push_back(*might_collide_at - s0);
    } else {
      collision_points_by_car.push_back(boost::none);
    }
  }
  return collision_points_by_car;
#endif
  ////  std::vector<boost::optional<T>> collision_points_by_car(num_cars_);
  ////  return collision_points_by_car;
}


template <typename T>
std::unique_ptr<systems::BasicVector<T>> EndlessRoadOracle<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadOracleOutput<T>>();
}

}  // namespace automotive
}  // namespace drake

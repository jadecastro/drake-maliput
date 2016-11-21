#include "decision_layer.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <Eigen/Geometry>

//#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
DecisionLayer<T>::DecisionLayer(
    const maliput::utility::InfiniteCircuitRoad* road,
    const int num_cars, const int num_targets_per_car)
    : road_(road), num_cars_(num_cars),
      num_targets_per_car_(num_targets_per_car) {
  // Declare an input for N-1 remaining cars in the world.
  this->DeclareInputPort(systems::kVectorValued,
                         EndlessRoadCarStateIndices::kNumCoordinates,
                         systems::kContinuousSampling);

  for (int i = 0; i < num_cars; ++i) {
    target_inports_.push_back(
        this->DeclareInputPort(systems::kVectorValued,
                               EndlessRoadCarStateIndices::kNumCoordinates,
                               systems::kContinuousSampling));
  }
  // Declare an output for the M target traffic cars of interest.
  for (int i = 0; i < num_targets_per_car; ++i) {
    outports_.push_back(
        this->DeclareOutputPort(systems::kVectorValued,
                                EndlessRoadOracleOutputIndices::kNumCoordinates,
                                systems::kContinuousSampling));
  }
}

template <typename T>
DecisionLayer<T>::~DecisionLayer() {}

template <typename T>
const systems::SystemPortDescriptor<T>&
DecisionLayer<T>::get_self_input_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
DecisionLayer<T>::get_world_input_port(const int i) const {
  return systems::System<T>::get_input_port(i+1);
}

template <typename T>
void DecisionLayer<T>::EvalOutput(const systems::Context<T>& context,
                                      systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

//DBG  std::cerr << "EvalOutput()  ORACLE " << std::endl;

  // Obtain the self-car input.
  const systems::BasicVector<T>* basic_input_self =
      this->EvalVectorInput(context, this->get_self_input_port().get_index());
  DRAKE_ASSERT(basic_input_self);
  const EndlessRoadCarState<T>* const input_self_car =
    dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_self);

  // Obtain the world-car inputs.
  std::vector<const EndlessRoadCarState<T>*> inputs_world;
  for (int i = 0; i < num_cars_; ++i) {
    const systems::BasicVector<T>* basic_input_world =
      this->EvalVectorInput(context,
                            this->get_world_input_port(i).get_index());
    DRAKE_ASSERT(basic_input_world);
    const EndlessRoadCarState<T>* const input_world =
      dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_world);
    DRAKE_ASSERT(input_world);
    inputs_world.push_back(input_world);
  }

  // Obtain the output pointers.
  std::vector<EndlessRoadOracleOutput<T>*> target_outputs;
  for (int i = 0; i < num_targets_per_car_; ++i) {
    EndlessRoadOracleOutput<T>* const output_vector =
        dynamic_cast<EndlessRoadOracleOutput<T>*>(
            output->GetMutableVectorData(i));
    DRAKE_ASSERT(output_vector);
    target_outputs.push_back(output_vector);
  }

  DoEvalOutput(input_self_car, inputs_world, target_outputs);
}

const double kEnormousDistance = 1e12;
const double kCarLength = 4.6;  // TODO(maddog) Get from somewhere else.
const double kPerceptionDistance = 30.0;  // Targets are imperceptible
                                          // outside this radius.
// TODO(jadecastro): Store these as IdmParameters or CarParameters.

template <typename T>
void DecisionLayer<T>::DoEvalOutput(
    const EndlessRoadCarState<T>* self_car_input,
    const std::vector<const EndlessRoadCarState<T>*>& world_car_inputs,
    std::vector<EndlessRoadOracleOutput<T>*>& target_outputs) const {
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

  // Time to look-ahead for intersections.
  // TODO(maddog) Should be a multiple of IDM headway h.
  // TODO(maddog) Should have a distance component, too, as a multiple of
  //              car lengths.
  //////////  const T kEventHorizon{2.0};  // seconds
  const T kEventHorizon{10.0};  // seconds

  //
  // We really only want the InfiniteCircuitRoad to serve two functions:
  //  1)  until proper hybrid system support is available, emulate a
  //      uni-modal continuous system for the vehicle control;
  //  2)  represent the path of the cars through the road network.
  // Here we extract the RoadPositions of each car in the underlying
  // source RoadGeometry, and we extract their paths.
  SourceState self_source_state;
  std::vector<SourceState> world_source_states;
  std::vector<PathRecord> self_car_path;
  UnwrapEndlessRoadCarState(self_car_input, world_car_inputs, road_,
                            kEventHorizon, &self_source_state,
                            &world_source_states, &self_car_path);

  AssessLongitudinal(self_source_state, world_source_states,
                     self_car_path, target_outputs);
  // AssessIntersections(source_states, paths, target_outputs);
}

template <typename T>
void DecisionLayer<T>::UnwrapEndlessRoadCarState(
    const EndlessRoadCarState<double>* self_car_input,
    const std::vector<const EndlessRoadCarState<double>*>& world_car_inputs,
    const maliput::utility::InfiniteCircuitRoad* road,
    const double horizon_seconds,
    SourceState* self_source_state,
    std::vector<SourceState>* world_source_states,
    std::vector<PathRecord>* self_car_path) const {

  //self_source_state->clear();
  world_source_states->clear();
  //self_car_path->clear();
  for (size_t i = 0; i < world_car_inputs.size()+1; ++i) {
    // NB(jadecastro): Not really meaningful to call it `self`
    // anymore, but whatevs.
    const EndlessRoadCarState<double>* self =
      (i == 0) ? self_car_input : world_car_inputs[i-1];
    const maliput::api::RoadPosition rp = road->ProjectToSourceRoad(
        {self->s(), 0., 0.}).first;
    // TODO(maddog)  Until we deal with cars going the wrong way.
    DRAKE_DEMAND(std::cos(self->heading()) >= 0.);
    DRAKE_DEMAND(self->speed() >= 0.);
    const double longitudinal_speed =
        self->speed() * std::cos(self->heading());

    if (i == 0) {
      self_source_state->rp = rp;
      self_source_state->longitudinal_speed = longitudinal_speed;
    } else {
      world_source_states->emplace_back(rp, longitudinal_speed);
    }

    const double horizon_meters = longitudinal_speed * horizon_seconds;
    // TODO(maddog)  Is this < constraint relevant anymore???
    DRAKE_DEMAND(horizon_meters < (0.5 * road->cycle_length()));
    DRAKE_DEMAND(horizon_meters >= 0.);
    const double circuit_s0 = road->lane()->circuit_s(self->s());

    int path_index = road->GetPathIndex(circuit_s0);
    maliput::utility::InfiniteCircuitRoad::Record path_record =
        road->path_record(path_index);
    double circuit_s_in = circuit_s0;
    while (circuit_s_in <= (circuit_s0 + horizon_meters)) {
      self_car_path->push_back({path_record.lane, path_record.is_reversed});

      // TODO(maddog) Index should decrement for s_dot < 0.
      if (++path_index >= road->num_path_records()) {
        path_index = 0;
      }
      path_record = road->path_record(path_index);
      circuit_s_in = path_record.start_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (circuit_s_in < circuit_s0) { circuit_s_in += road->cycle_length(); }
    }
    DRAKE_DEMAND(!self_car_path->empty());
  }
}

template <typename T>
void DecisionLayer<T>::AssessLongitudinal(
            const SourceState& self_source_states,
            const std::vector<SourceState>& world_source_states,
            std::vector<PathRecord>& self_car_path,
            std::vector<EndlessRoadOracleOutput<double>*>& target_outputs)
  const {

  // Seed the output with infinite obstacles.
  for (int i = 0; i < (int) target_outputs.size(); ++i) {
    target_outputs[i]->set_net_delta_sigma(kEnormousDistance);
    target_outputs[i]->set_delta_sigma_dot(0.);
  }

  // Calculate longitudinal position of each car in the circuit.
  // Sort cars by longitudinal position:  use an ordered multimap that
  // maps s-position --> car-index.

  // Map{Lane* --> MultiMap{s --> car_index}}
  std::map<const maliput::api::Lane*,
           std::multimap<double, int>> cars_by_lane_and_s;

  for (int i = 0; i < (int) world_source_states.size(); ++i) {
    const SourceState& target = world_source_states[i];
    cars_by_lane_and_s[target.rp.lane].emplace(target.rp.pos.s, i);
  }

  // For each car:
  //  - find nearest other car that is:
  //     a) ahead of self, and
  //     b) has a lateral position within XXXXXX bounds of self.
  //  - compute/record:
  //     * net true longitudinal distance to other car (delta sigma)
  //     * true longitudinal velocity difference (delta sigma-dot)
  // TODO(maddog)  Do something reasonable if num_cars_ < 2.
  //
  // TODO(jadecastro): I think `delta-sigma` is a misuse of the notation
  // - it should be `delta-s` or similar.

  const SourceState& self = self_source_states;
  const maliput::api::RoadPosition self_rp = self.rp;

  // NB(jadecastro): Defaults the targets as static obstacles at infinity.
  double delta_position = kEnormousDistance;
  double delta_velocity = 0.;

  // Find the next car which is ahead of self, by at least a car-length.
  // Search along the sequence of lanes to be taken by self, starting with
  // the current lane.
  DRAKE_DEMAND(self_rp.lane == self_car_path[0].lane);

  // TODO(maddog)  Review this; the notion is to skip self, and might as
  //               well skip any cars sooo close to self.
  double bound_nudge = 0.1 * kCarLength;
  bool is_first = true;
  // NB(jadecastro): Start at the current lane for self (car i);
  // return the data for the next car in the train.
  auto path_it = self_car_path.begin();
  // TODO(jadecastro): generalize this to encompass the semicircle
  // ahead of the self-car.
  double s_prev;  // The linear position of the last car found in
                  // the train, starting with the self car.
  maliput::api::RoadPosition next_rp;
  double sum = 0.;
  for (int i = 0; i < (int) target_outputs.size(); ++i) {
    if (sum > kPerceptionDistance) { break; }
    // Only consider cars in front of `self`, along the prescribed path.
    if (!path_it->is_reversed) {
      const double s0 = (is_first) ? self_rp.pos.s : 0.;
      s_prev = (is_first) ? s0 : next_rp.pos.s;
      const double lane_length = path_it->lane->length() - s0;
      DRAKE_DEMAND(lane_length > 0.);
      // NB(jadecastro): `next_it` contains states and index of the
      // next car in the train.
      auto next_it = cars_by_lane_and_s[path_it->lane].upper_bound(
                                                   s_prev + bound_nudge);
      if (next_it != cars_by_lane_and_s[path_it->lane].end()) {
        const SourceState& next = world_source_states[next_it->second];
        next_rp = next.rp;
        delta_position = sum + (next_rp.pos.s - s0);
        if (delta_position < 0.) {
          std::cerr << "FOR dp " << delta_position
                    << "   sum " << sum
                    << "   next-s " << next_rp.pos.s
                    << "   s0 " << s0
                    << "   first " << is_first
                    << std::endl;
        }
        // TODO(maddog)  Oy, this needs to account for travel direction
        //               of next in its source lane, not inf-circuit lane.
        delta_velocity = self.longitudinal_speed - next.longitudinal_speed;
        break;
      }
      sum += lane_length;
      bound_nudge = std::max(0., bound_nudge - lane_length);
    } else {
      /* Self is travelling this lane backwards. */
      const double s0 = (is_first) ? self_rp.pos.s : path_it->lane->length();
      s_prev = (is_first) ? s0 : next_rp.pos.s;
      const double lane_length = s0;
      DRAKE_DEMAND(lane_length > 0.);

      auto next_it = std::make_reverse_iterator(
          cars_by_lane_and_s[path_it->lane].lower_bound(
                                                        s_prev - bound_nudge));
      if (next_it != cars_by_lane_and_s[path_it->lane].rend()) {
        const SourceState& next = world_source_states[next_it->second];
        next_rp = next.rp;
        delta_position = sum + (s0 - next_rp.pos.s);
        if (delta_position < 0.) {
          std::cerr << "REV dp " << delta_position
                    << "   sum " << sum
                    << "   next-s " << next_rp.pos.s
                    << "   s0 " << s0
                    << "   first " << is_first
                    << std::endl;
        }
        // TODO(maddog)  Oy, this needs to account for travel direction
        //               of next in its source lane, not inf-circuit lane.
        delta_velocity = self.longitudinal_speed - next.longitudinal_speed;
        break;
      }
      sum += lane_length;
      bound_nudge = std::max(0., bound_nudge - lane_length);
    }

    ++path_it;
    is_first = false;

    // While within horizon...
    //   Traipse ahead to next branchpoint.
    //   Loop over all other-lanes on same side of BP as this-lane.
    //     Loop over all cars on lane
    //       If car is not heading toward BP, skip it.
    //       If car is closer to BP (farther from self, in parallel)
    //         than last best other, skip it.
    //       Mark down car as "best other" at X distance from BP.
    //     ...Recurse back into lanes leading to this lane, too.

    // TODO(maddog) Do a correct distance measurement (not just delta-s).
    const double net_delta_sigma = delta_position - kCarLength;
    if (net_delta_sigma <= 0.) {
      std::cerr << "TOO CLOSE!      delta_pos " << delta_position
                << "   car len " << kCarLength
                << "   nds " << net_delta_sigma
                << std::endl;
    }
    // If delta_position < kCarLength, the cars crashed!
    // TODO(maddog)  Or, they were passing, with lateral distance > width....
    DRAKE_DEMAND(net_delta_sigma > 0.);

    // Populate the relative target quantities.
    target_outputs[i]->set_net_delta_sigma(net_delta_sigma);
    target_outputs[i]->set_delta_sigma_dot(delta_velocity);
  }
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> DecisionLayer<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadOracleOutput<T>>();
}

// These instantiations must match the API documentation in
// decision_layer.h.
// TODO(jadecastro): AutoDiff, symbolic_expression
template class DecisionLayer<double>;

}  // namespace automotive
}  // namespace drake

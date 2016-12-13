#include "target_selector_and_idm_merge_planner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

#include <Eigen/Geometry>

//#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
//#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

// Debugging... go through these!!!!
//#include "drake/automotive/maliput/api/car_data.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

namespace drake {
namespace automotive {

using maliput::api::RoadPosition;
using maliput::api::LanePosition;

template <typename T>
TargetSelectorAndIdmMergePlanner<T>::TargetSelectorAndIdmMergePlanner(
    const maliput::utility::InfiniteCircuitRoad* road, const int num_cars,
    const int num_targets_per_car, const bool do_restrict_to_lane,
    const bool do_sort)
    : road_(road),
      num_cars_(num_cars),
      num_targets_per_car_(num_targets_per_car),
      do_restrict_to_lane_(do_restrict_to_lane),
      do_sort_(do_sort) {
  // Fail fast if we are asking the impossible.
  DRAKE_DEMAND(num_targets_per_car <= num_cars - 1);
  // Declare an input for the self car.
  this->DeclareInputPort(systems::kVectorValued, 4,
                         systems::kContinuousSampling);
  // Declare an input for N-1 remaining cars in the world.
  for (int i = 0; i < num_cars - 1; ++i) {
    this->DeclareInputPort(systems::kVectorValued, 4,
                           systems::kContinuousSampling);
  }
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          2, /* Longitudinal and lateral acceleration. */
                          systems::kContinuousSampling);
}

template <typename T>
TargetSelectorAndIdmMergePlanner<T>::~TargetSelectorAndIdmMergePlanner() {}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelectorAndIdmMergePlanner<T>::get_self_inport() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelectorAndIdmMergePlanner<T>::get_world_inport(const int i) const {
  DRAKE_DEMAND(i < num_cars_ - 1);
  return systems::System<T>::get_input_port(i + 1);
}

template <typename T>
void TargetSelectorAndIdmMergePlanner<T>::EvalOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the self-car input.
  const systems::BasicVector<T>* basic_input_self =
      this->EvalVectorInput(context, this->get_self_inport().get_index());
  DRAKE_ASSERT(basic_input_self);
  // const EndlessRoadCarState<T>* const input_self_car =
  //  dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_self);
  // DRAKE_ASSERT(input_self_car);
  std::cerr << "  self position: " << basic_input_self->GetAtIndex(0) << "\n";

  // Obtain the world-car inputs.
  // std::vector<const EndlessRoadCarState<T>*> inputs_world;
  std::vector<const systems::BasicVector<T>*> inputs_world;
  for (int i = 0; i < num_cars_ - 1; ++i) {
    const systems::BasicVector<T>* basic_input_world =
        this->EvalVectorInput(context, this->get_world_inport(i).get_index());
    DRAKE_ASSERT(basic_input_world);
    // const EndlessRoadCarState<T>* const input_world =
    //  dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_world);
    // DRAKE_ASSERT(input_world);
    // inputs_world.push_back(input_world);
    std::cerr << "  world car " << i
              << " position: " << basic_input_world->GetAtIndex(0) << "\n";

    inputs_world.push_back(basic_input_world);
  }

  // Obtain the output pointer.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  std::vector<CarData> car_data_targets;
  CarData car_data_self =
      SelectCarState(basic_input_self, inputs_world, &car_data_targets);

  // Compute the IDM accelerations.
  ComputeIdmAccelerations(car_data_self, car_data_targets, context,
                          output_vector);
}

const double kEnormousDistance = 1e12;
const double kPerceptionDistance = 60.0;  // Targets are imperceptible
                                          // outside this radius.
const double kHorizonSeconds = 10.;
// TODO(jadecastro): Store these as IdmParameters or CarParameters.

template <typename T>
void TargetSelectorAndIdmMergePlanner<T>::UnwrapEndlessRoadCarState(
    const SourceState& source_states_self, const double& s_absolute,
    const maliput::utility::InfiniteCircuitRoad& road,
    std::vector<PathRecord>* path_self_car) const {
  const double horizon_meters =
      source_states_self.longitudinal_speed * kHorizonSeconds;
  DRAKE_DEMAND(horizon_meters >= 0.);

  std::cerr << "   %%% source_states_self.rp.pos.s:"
            << source_states_self.rp.pos.s << std::endl;
  std::cerr << "   %%% s_absolute:" << s_absolute << std::endl;
  const double circuit_s0 = road.lane()->circuit_s(s_absolute);

  int path_index = road.GetPathIndex(circuit_s0);
  std::cerr << "   %%% path_index:" << path_index << std::endl;
  maliput::utility::InfiniteCircuitRoad::Record path_record =
      road.path_record(path_index);
  double circuit_s_in = circuit_s0;
  while (circuit_s_in <= (circuit_s0 + horizon_meters)) {
    path_self_car->push_back({path_record.lane, path_record.is_reversed});

    // TODO(maddog) Index should decrement for s_dot < 0.
    if (++path_index >= road.num_path_records()) {
      path_index = 0;
    }
    path_record = road.path_record(path_index);
    circuit_s_in = path_record.start_circuit_s;
    // Handle wrap-around of "circuit s" values.
    if (circuit_s_in < circuit_s0) {
      circuit_s_in += road.cycle_length();
    }
  }
  DRAKE_DEMAND(!path_self_car->empty());
}

template <typename T>
std::pair<double, double>
TargetSelectorAndIdmMergePlanner<T>::AssessLongitudinal(
    const IdmPlannerParameters<T>& params,
    const SourceState& source_states_self,
    const std::vector<SourceState>& source_states_targets,
    const std::vector<PathRecord>& path_self_car) const {
  // NB(jadecastro): Defaults the targets as static obstacles at infinity.
  double delta_position = kEnormousDistance;
  double delta_velocity = 0.;

  const SourceState& self_car = source_states_self;
  //`const maliput::api::RoadPosition rp_self_car = self_car.rp;

  // Find the next car which is ahead of self, by at least a car-length.
  // Search along the sequence of lanes to be taken by self, starting with
  // the current lane.
  // DRAKE_DEMAND(rp_self_car.lane == path_self_car[0].lane);

  // TODO(jadecastro): generalize this to encompass the semicircle
  // ahead of the self-car.  Will this allow us to generalize IDM to 2D?
  RoadPosition rp_this_car;
  auto lane_this_car = path_self_car.begin();  // Predefined path
                                               // traversed by the
                                               // self-car.

  // Map{Lane* --> MultiMap{s --> car_index}}
  std::map<const maliput::api::Lane*, std::multimap<double, int>>
      cars_by_lane_and_s;
  DRAKE_DEMAND(num_targets_per_car_ == (int)source_states_targets.size());
  for (int i = 0; i < num_targets_per_car_; ++i) {
    const SourceState& target = source_states_targets[i];
    cars_by_lane_and_s[target.rp.lane].emplace(target.rp.pos.s, i);
  }

  // NB(jadecastro): Starting at the current self car lane, crawl
  // through the segments and return the data for the next car in
  // the train.
  double lane_length_sum = 0.;  // Sum of the segment lengths crawled
                                // through so far.
  bool is_first = true;
  double bound_nudge = 0.1 * params.car_length();
  // Loop over the partial path constructed over the perception horizon.
  while (lane_this_car != path_self_car.end()) {
    std::cerr << "        ** lane_this_car: " << lane_this_car->lane
              << std::endl;
    std::cerr << "          !lane_this_car->is_reversed: "
              << !lane_this_car->is_reversed << std::endl;
    std::cerr << "          is_first: " << is_first << std::endl;
    const double possible_lane_datum =
        (!lane_this_car->is_reversed) ? 0. : lane_this_car->lane->length();
    const double lane_datum =
        (is_first) ? self_car.rp.pos.s : possible_lane_datum;
    const double lane_length = (!lane_this_car->is_reversed)
                                   ? lane_this_car->lane->length() - lane_datum
                                   : lane_datum;
    DRAKE_DEMAND(lane_length > 0.);
    // The linear position of the last car found in the train
    const double last_position = (is_first) ? lane_datum : rp_this_car.pos.s;

    // NB(jadecastro): `states_this_car` contains states and index of the
    // next car in the train.
    bool is_car_past_limit;
    int index_this_car;
    if (!lane_this_car->is_reversed) {
      auto states_this_car =
          cars_by_lane_and_s[lane_this_car->lane].upper_bound(last_position +
                                                              bound_nudge);
      auto lane_limit = cars_by_lane_and_s[lane_this_car->lane].end();
      is_car_past_limit = states_this_car != lane_limit;
      // if (is_first) {
      //  is_car_past_limit |=
      //     (states_this_car->first - self_car.rp.pos.s) >= 0;
      //}
      index_this_car = states_this_car->second;
      std::cerr << "      index_this_car: " << index_this_car << std::endl;
    } else {
      std::cerr << "      lane_length: " << lane_length << std::endl;
      auto states_this_car =
          cars_by_lane_and_s[lane_this_car->lane].lower_bound(last_position -
                                                              bound_nudge);
      std::cerr << "      states_this_car: " << states_this_car->first
                << std::endl;
      auto lane_limit = cars_by_lane_and_s[lane_this_car->lane].end();
      is_car_past_limit = states_this_car != lane_limit;
      // if (is_first) {
      //  is_car_past_limit |=
      //     (self_car.rp.pos.s - states_this_car->first) >= 0;
      //}
      index_this_car = states_this_car->second;
      std::cerr << "      index_this_car: " << index_this_car << std::endl;
    }

    // TODO (jadecastro): This could cause false alarms and lead to
    // the sim crashing inadvertantly.
    if (is_car_past_limit) {
      const SourceState& this_car = source_states_targets[index_this_car];
      rp_this_car = this_car.rp;
      // TODO (jadecastro): This is somewhat of a hack.
      bool is_not_reversed_nor_infty =
          !lane_this_car->is_reversed ||
          (rp_this_car.pos.s > 0.5 * kEnormousDistance);
      const double pos_relative_to_lane =
          (is_not_reversed_nor_infty) ? (rp_this_car.pos.s - lane_datum)
                                      : (lane_datum - rp_this_car.pos.s);
      // Compute the relative position and velocity of the next car found.
      delta_position =
          lane_length_sum + pos_relative_to_lane - params.car_length();
      std::cerr << "      rp_this_car.pos.s: " << rp_this_car.pos.s
                << std::endl;
      std::cerr << "      lane_length_sum: " << lane_length_sum << std::endl;
      std::cerr << "      pos_relative_to_lane: " << pos_relative_to_lane
                << std::endl;
      std::cerr << "      lane_datum: " << lane_datum << std::endl;
      std::cerr << "      lane_length: " << lane_length << std::endl;
      delta_velocity =
          self_car.longitudinal_speed - this_car.longitudinal_speed;
      break;
    }

    lane_length_sum += lane_length;
    bound_nudge = std::max(0., bound_nudge - lane_length);

    ++lane_this_car;
    is_first = false;
  }  // while (lane_this_car != path_self_car.end())

  // TODO(maddog) Do a correct distance measurement (not just delta-s).
  // TODO(jadecastro): kCarLength should be a Parameter of type T.
  if (delta_position <= 0.) {
    std::cerr << "TOO CLOSE!      delta_pos " << delta_position << std::endl;
  }
  // If delta_position < kCarLength, the cars crashed!
  DRAKE_DEMAND(delta_position > 0.);
  std::cerr << "  @@@@@ IdmPlanner  delta_position: " << delta_position
            << std::endl;
  std::cerr << "  @@@@@ IdmPlanner  delta_velocity: " << delta_velocity
            << std::endl;
  // Populate the relative target quantities.

  return std::make_pair(delta_position, delta_velocity);
}

template <typename T>
typename TargetSelectorAndIdmMergePlanner<T>::CarData
TargetSelectorAndIdmMergePlanner<T>::SelectCarState(
    const systems::BasicVector<T>* input_self_car,  // TODO(jadecastro): Ref.
    const std::vector<const systems::BasicVector<T>*>& inputs_world_car,
    std::vector<CarData>* car_data_targets) const {
  car_data_targets->clear();

  std::vector<double> distances;
  // std::pair<T, T> pair;
  std::vector<CarData> car_data;

  const systems::BasicVector<double>* car_state_self = input_self_car;
  const double long_pos_road_self = car_state_self->GetAtIndex(0);
  const double heading_road_self = car_state_self->GetAtIndex(2);
  const double vel_forward_self = car_state_self->GetAtIndex(3);

  const maliput::api::RoadPosition rp_self =
      road_->ProjectToSourceRoad({long_pos_road_self, 0., 0.}).first;
  DRAKE_DEMAND(std::cos(heading_road_self) >= 0.);
  DRAKE_DEMAND(vel_forward_self >= 0.);
  maliput::api::GeoPosition geo_pos_self =
      rp_self.lane->ToGeoPosition(rp_self.pos);
  // Seed the output with "infinite" obstacles.  TODO
  // (jadecastro): Is the lane currently occupied by the
  // self-car the best proxy to use here?
  const double longitudinal_speed =  // Along-lane speed.
      vel_forward_self * std::cos(heading_road_self);
  // pair = std::make_pair(T{rp.pos.s}, T{longitudinal_speed});
  const CarData car_data_self = {long_pos_road_self, rp_self.pos.s,
                                 longitudinal_speed, rp_self.lane};
  std::cerr << "   %%% long_pos_road_self:" << long_pos_road_self << std::endl;

  DRAKE_DEMAND(num_cars_ == (int)inputs_world_car.size() + 1);
  for (int i = 0; i < num_cars_ - 1; ++i) {
    const systems::BasicVector<double>* car_state = inputs_world_car[i];
    const double long_pos_road = car_state->GetAtIndex(0);
    const double heading_road = car_state->GetAtIndex(2);
    const double vel_forward = car_state->GetAtIndex(3);
    const maliput::api::RoadPosition rp =
        road_->ProjectToSourceRoad({long_pos_road, 0., 0.}).first;
    DRAKE_DEMAND(std::cos(heading_road) >= 0.);
    DRAKE_DEMAND(vel_forward >= 0.);

    // Ignore any cars outside the perception distance.
    if (!do_restrict_to_lane_) {
      maliput::api::GeoPosition geo_pos = rp.lane->ToGeoPosition(rp.pos);
      distances.emplace_back(
          road_->RoadGeometry::Distance(geo_pos_self, geo_pos));
      if (distances[i] < kPerceptionDistance) {
        const double longitudinal_speed =  // Along-lane speed.
            vel_forward * std::cos(heading_road);
        // pair = std::make_pair(T{rp.pos.s}, T{longitudinal_speed});
        // car_data.emplace_back(std::make_pair(&pair, rp.lane));
        CarData car_data_value(long_pos_road, rp.pos.s, longitudinal_speed,
                               rp.lane);
        car_data.emplace_back(car_data_value);
      } else {
        double position_infty = kEnormousDistance;
        CarData car_data_infty(position_infty, position_infty, 0., rp.lane);
        car_data.emplace_back(car_data_infty);
      }
    } else {
      DRAKE_ABORT();
      // TODO(jadecastro): Fill me in......
    }
  }

  // Populate the output ports with data for each of the registered targets.
  std::vector<int> index_set(
      distances.size());  // TODO(jadecastro): Must be a better way.
  for (int i = 0; i < (int)distances.size(); ++i) {
    index_set[i] = i;
  }
  const std::vector<int> possibly_sorted_indices =
      (do_sort_) ? SortDistances(distances) : index_set;
  for (auto i : possibly_sorted_indices) {
    if (i >= num_targets_per_car_) {
      break;
    }
    car_data_targets->emplace_back(car_data[i]);
  }
  return car_data_self;
}

template <typename T>
void TargetSelectorAndIdmMergePlanner<T>::ComputeIdmAccelerations(
    const CarData& car_data_self, const std::vector<CarData>& car_data_targets,
    const systems::Context<T>& context,
    systems::BasicVector<T>* output_vector) const {
  // Obtain the parameters.
  const int kParamsIndex = 0;
  const IdmPlannerParameters<T>& params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kParamsIndex);
  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();
  const T& car_length = params.car_length();

  const T& s_self_absolute = car_data_self.s;  // Road position
  const T& s_self = car_data_self.lane_s;      // Lane position
  const T& v_self = car_data_self.lane_v;      // Along-lane velocity
  const maliput::api::Lane* lane_self = car_data_self.lane;
  // Compose the inputs into the required containers.
  LanePosition lp_self(s_self, 0., 0.);
  RoadPosition rp_self(lane_self, lp_self);
  SourceState source_states_self(rp_self, v_self);

  std::vector<SourceState> source_states_targets;
  for (auto car_data : car_data_targets) {
    const T& s_target = car_data.lane_s;
    const T& v_target = car_data.lane_v;
    const maliput::api::Lane* lane_target = car_data.lane;
    LanePosition lp_target(s_target, 0., 0.);
    RoadPosition rp_target(lane_target, lp_target);
    source_states_targets.emplace_back(rp_target, v_target);
  }

  // Get the local path of the self-car.
  std::vector<PathRecord> path_self_car;
  UnwrapEndlessRoadCarState(source_states_self, s_self_absolute, *road_,
                            &path_self_car);

  // Obtain the relative quantities for the nearest car ahead of the self-car.
  std::pair<double, double> relative_sv = AssessLongitudinal(
      params, source_states_self, source_states_targets, path_self_car);
  const double s_rel = relative_sv.first;
  const double v_rel = relative_sv.second;

  // Check that we're supplying the planner with sane parameters and
  // inputs.
  DRAKE_DEMAND(a > 0.0);
  DRAKE_DEMAND(b > 0.0);
  DRAKE_DEMAND(s_rel > car_length);

  const T s_star =
      s_0 + v_self * time_headway + v_self * v_rel / (2 * sqrt(a * b));

  std::cerr << "  IdmPlanner v_ref: " << v_ref << std::endl;
  std::cerr << "  IdmPlanner s_self: " << s_self << std::endl;
  std::cerr << "  IdmPlanner v_self: " << v_self << std::endl;
  std::cerr << "  IdmPlanner s_rel: " << s_rel << std::endl;
  std::cerr << "  IdmPlanner v_rel: " << v_rel << std::endl;

  output_vector->SetAtIndex(
      0, a * (1.0 - pow(v_self / v_ref, delta) -
              pow(s_star / s_rel, 2.0)));  // Longitudinal acceleration.
  output_vector->SetAtIndex(1, 0.0);       // Lateral acceleration.

  std::cerr << "  IdmPlanner accel cmd: " << output_vector->GetAtIndex(0)
            << std::endl;
  std::cerr << "  $$$$$$$$ Selector/IdmPlanner::ComputeIdmAccelerations."
            << std::endl;
}

// Lambda-expression approach to sort indices (adapted from
// http://stackoverflow.com/questions/1577475/ ...
//   c-sorting-and-keeping-track-of-indexes)
template <typename T>
std::vector<int> TargetSelectorAndIdmMergePlanner<T>::SortDistances(
    const std::vector<T>& v) const {
  // initialize original index locations
  std::vector<int> indices(v.size());
  std::iota(indices.begin(), indices.end(), 0);
  sort(indices.begin(), indices.end(),
       [&v](int i1, int i2) { return v[i1] < v[i2]; });
  return indices;
}

template <typename T>
std::unique_ptr<systems::Parameters<T>>
TargetSelectorAndIdmMergePlanner<T>::AllocateParameters() const {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  // TODO(jadecastro): Workaround to ignore all input arguments since
  // they don't seem to be working in the simulator.
  params->set_v_ref(T(30.));         // desired velocity in free traffic. (30)
  params->set_a(T(4.0));             // max acceleration.
  params->set_b(T(12.0));            // comfortable braking deceleration.
  params->set_s_0(T(2.0));           // minimum desired net distance.
  params->set_time_headway(T(1.0));  // desired time headway to lead vehicle.
  params->set_delta(T(4.0));       // recommended choice of free-road exponent.
  params->set_car_length(T(4.6));  // length of leading car.
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

// These instantiations must match the API documentation in
// target_selector.h.
// TODO(jadecastro): AutoDiff, symbolic_expression
template class TargetSelectorAndIdmMergePlanner<double>;

}  // namespace automotive
}  // namespace drake

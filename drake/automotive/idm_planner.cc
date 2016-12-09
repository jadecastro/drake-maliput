#include "drake/automotive/idm_planner.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

// Debugging... go through these!!!!
#include "drake/automotive/maliput/api/car_data.h"
//#include "drake/automotive/maliput/api/lane_data.h"
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
using maliput::api::CarData;

template <typename T>
IdmPlanner<T>::IdmPlanner(
       const maliput::utility::InfiniteCircuitRoad* road,
         const T& v_ref, const int num_targets_per_car)
    : road_(road), v_ref_(v_ref),
      num_targets_per_car_(num_targets_per_car) {
  // TODO(jadecastro): Remove v_ref from the constructor.
  // The reference velocity must be strictly positive.
  DRAKE_ASSERT(v_ref > 0);

  // Declare the self car input port.
  this->DeclareAbstractInputPort(systems::kContinuousSampling);
  // Declare the traffic car input ports.
  for (int i = 0; i < num_targets_per_car; ++i) {
    this->DeclareAbstractInputPort(systems::kContinuousSampling);
  }
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          2,  // Longitudinal and lateral acceleration.
                          systems::kContinuousSampling);
}

template <typename T>
IdmPlanner<T>::~IdmPlanner() {}

template <typename T>
const systems::SystemPortDescriptor<T>& IdmPlanner<T>::get_self_inport() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
IdmPlanner<T>::get_target_inport(const int i) const {
  DRAKE_DEMAND(i < num_targets_per_car_);
  return systems::System<T>::get_input_port(i+1);
}

const double kEnormousDistance = 1e12;
const double kCarLength = 4.6;
const double kHorizonSeconds = 10.;
// TODO(jadecastro): Store these as IdmParameters or CarParameters.

template <typename T>
void IdmPlanner<T>::UnwrapEndlessRoadCarState(
    const SourceState& source_state_self,
    const maliput::utility::InfiniteCircuitRoad& road,
    std::vector<PathRecord>* path_self_car) const {

  const double horizon_meters =
    source_state_self.longitudinal_speed * kHorizonSeconds;
  DRAKE_DEMAND(horizon_meters >= 0.);

  const maliput::api::LanePosition position = source_state_self.rp.pos;
  const double circuit_s0 = road.lane()->circuit_s(position.s);

  int path_index = road.GetPathIndex(circuit_s0);
  maliput::utility::InfiniteCircuitRoad::Record path_record =
    road.path_record(path_index);
  double circuit_s_in = circuit_s0;
  while (circuit_s_in <= (circuit_s0 + horizon_meters)) {
    path_self_car->push_back({path_record.lane, path_record.is_reversed});

    // TODO(maddog) Index should decrement for s_dot < 0.
    if (++path_index >= road.num_path_records()) { path_index = 0; }
    path_record = road.path_record(path_index);
    circuit_s_in = path_record.start_circuit_s;
    // Handle wrap-around of "circuit s" values.
    if (circuit_s_in < circuit_s0) { circuit_s_in += road.cycle_length(); }
  }
  DRAKE_DEMAND(!path_self_car->empty());
}

template <typename T>
std::pair<double, double> IdmPlanner<T>::AssessLongitudinal(
            const IdmPlannerParameters<T>& params,
            const SourceState& source_state_self,
            const std::vector<SourceState>& source_states_target,
            const std::vector<PathRecord>& path_self_car)
  const {

 // NB(jadecastro): Defaults the targets as static obstacles at infinity.
  double delta_position = kEnormousDistance;
  double delta_velocity = 0.;

  const SourceState& self_car = source_state_self;
  const maliput::api::RoadPosition rp_self_car = self_car.rp;

  // Find the next car which is ahead of self, by at least a car-length.
  // Search along the sequence of lanes to be taken by self, starting with
  // the current lane.
  DRAKE_DEMAND(rp_self_car.lane == path_self_car[0].lane);

  // TODO(jadecastro): generalize this to encompass the semicircle
  // ahead of the self-car.  Will this allow us to generalize IDM to 2D?
  RoadPosition rp_this_car;
  auto lane_this_car = path_self_car.begin();  // Predefined path
                                                     // traversed by the
                                                     // self-car.

  // Map{Lane* --> MultiMap{s --> car_index}}
  std::map<const maliput::api::Lane*,
           std::multimap<double, int>> cars_by_lane_and_s;
  DRAKE_DEMAND(num_targets_per_car_ == (int) source_states_target.size());
  for (int i = 0; i < num_targets_per_car_; ++i) {
    const SourceState& target = source_states_target[i];
    cars_by_lane_and_s[target.rp.lane].emplace(target.rp.pos.s, i);
  }

  // NB(jadecastro): Starting at the current self car lane, crawl
  // through the segments and return the data for the next car in
  // the train.
  double lane_length_sum = 0.;  // Sum of the segment lengths crawled
                            // through so far.
  bool is_first = true;
  double bound_nudge = 0.1 * params.car_length();
  while (lane_this_car != path_self_car.end()) {
    const double possible_lane_datum =
        (!lane_this_car->is_reversed) ? 0. : lane_this_car->lane->length();
    const double lane_datum = (is_first) ? self_car.rp.pos.s :
        possible_lane_datum;
    const double lane_length =
        (!lane_this_car->is_reversed) ?
        lane_this_car->lane->length() - lane_datum : lane_datum;
    DRAKE_DEMAND(lane_length > 0.);

    // The linear position of the last car found in the train
    const double last_position = rp_this_car.pos.s;

    // NB(jadecastro): `states_this_car` contains states and index of the
    // next car in the train.
    bool is_car_past_limit;
    int index_this_car;
    if (!lane_this_car->is_reversed) {
      auto states_this_car =
          cars_by_lane_and_s[lane_this_car->lane].upper_bound(
              last_position + bound_nudge);
      auto lane_limit =
          cars_by_lane_and_s[lane_this_car->lane].end();
      is_car_past_limit = states_this_car != lane_limit;
      index_this_car = states_this_car->second;
    } else {
      auto states_this_car = std::make_reverse_iterator(
          cars_by_lane_and_s[lane_this_car->lane].lower_bound(
              last_position - bound_nudge));
      auto lane_limit = cars_by_lane_and_s[lane_this_car->lane].rend();
      is_car_past_limit = states_this_car != lane_limit;
      index_this_car = states_this_car->second;
    }

    if (!is_car_past_limit) {
      const SourceState& this_car =
          source_states_target[index_this_car];
      rp_this_car = this_car.rp;
      delta_position = lane_length_sum + (rp_this_car.pos.s - lane_datum)
        - params.car_length();
      // TODO(maddog)  Oy, this needs to account for travel direction
      //               of next in its source lane, not inf-circuit lane.
      delta_velocity = self_car.longitudinal_speed -
          this_car.longitudinal_speed;
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
    std::cerr << "TOO CLOSE!      delta_pos " << delta_position
              << std::endl;
  }
  // If delta_position < kCarLength, the cars crashed!
  DRAKE_DEMAND(delta_position > 0.);
  //std::cerr << "  @@@@@ IdmPlanner  net_delta_sigma: " <<
  //    net_delta_sigma << std::endl;
  //std::cerr << "  @@@@@ IdmPlanner  delta_velocity: " <<
  //    delta_velocity << std::endl;
  // Populate the relative target quantities.

  return std::make_pair(delta_position, delta_velocity);
}

template <typename T>
void IdmPlanner<T>::EvalOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  std::cerr << "  $$$$$$$$ IdmPlanner::EvalOutput ..." << std::endl;

  // Obtain the input/output structures we need to read from and write into.
  std::cerr << "  IdmPlanner EvalAbsrtactInput ..." << std::endl;
  const systems::AbstractValue* input_self =
    this->EvalAbstractInput(context, this->get_self_inport().get_index());
  std::cerr << "  IdmPlanner EvalAbsrtactInput." << std::endl;
  std::vector<const systems::AbstractValue*> inputs_target;
  for (int i = 0; i < num_targets_per_car_; ++i) {
    inputs_target.emplace_back(
       this->EvalAbstractInput(context,this->get_target_inport(i).get_index()));
  }
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

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

  const CarData& car_data_self = input_self->GetValue<CarData>();
  const T& s_self = car_data_self.s();  // Road position
  const T& v_self = car_data_self.v();  // Along-lane velocity
  const maliput::api::Lane* lane_self = &car_data_self.lane();
  // Compose the inputs into the required containers.
  LanePosition lp_self(s_self, 0., 0.);
  RoadPosition rp_self(lane_self, lp_self);
  SourceState source_state_self(rp_self, v_self);

  std::vector<SourceState> source_states_target;
  for (auto input_target : inputs_target) {
    const CarData& car_data_target = input_target->GetValue<CarData>();
    const T& s_target = car_data_target.s();
    const T& v_target = car_data_target.v();
    const maliput::api::Lane* lane_target = &car_data_target.lane();
    LanePosition lp_target(s_target, 0., 0.);
    RoadPosition rp_target(lane_target, lp_target);
    source_states_target.emplace_back(rp_target, v_target);
  }

  // Instantiate a container capturing the local path of the self-car.
  std::vector<PathRecord> path_self_car;
  UnwrapEndlessRoadCarState(source_state_self, *road_, &path_self_car);

  // Obtain the relative quantities for the nearest car ahead of the self-car.
  std::pair<double, double> relative_sv = AssessLongitudinal(params,
      source_state_self, source_states_target, path_self_car);
  const double s_rel = relative_sv.first;
  const double v_rel = relative_sv.second;

  // TODO (jadecastro): Wrap the IDM computation into its own function.
  // Check that we're supplying the planner with sane parameters and
  // inputs.
  DRAKE_DEMAND(a > 0.0);
  DRAKE_DEMAND(b > 0.0);
  DRAKE_DEMAND(s_rel > car_length);

  const T s_star = s_0 + v_self * time_headway
      + v_self * v_rel / (2 * sqrt(a * b));

  std::cerr << "  IdmPlanner v_ref: " << v_ref << std::endl;
  std::cerr << "  IdmPlanner s_self: " << s_self << std::endl;
  std::cerr << "  IdmPlanner v_self: " << v_self << std::endl;
  std::cerr << "  IdmPlanner s_rel: " << s_rel << std::endl;
  std::cerr << "  IdmPlanner v_rel: " << v_rel << std::endl;

  output_vector->SetAtIndex(
      0, a * (1.0 - pow(v_self / v_ref, delta) -
              pow( s_star / s_rel, 2.0)));  // Longitudinal acceleration.
  output_vector->SetAtIndex(1, 0.0);  // Lateral acceleration.

  std::cerr << "  IdmPlanner accel cmd: " <<
      output_vector->GetAtIndex(0) << std::endl;
  std::cerr << "  $$$$$$$$ IdmPlanner::EvalOutput." << std::endl;
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> IdmPlanner<T>::AllocateParameters()
    const {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  // TODO(jadecastro): Workaround to ignore all input arguments since
  // they don't seem to be working in the simulator.
  params->set_v_ref(T(30.));         // desired velocity in free traffic. (30)
  //params->set_v_ref(v_ref_);         // desired velocity in free traffic. (30)
  params->set_a(T(4.0));             // max acceleration.
  params->set_b(T(12.0));            // comfortable braking deceleration.
  params->set_s_0(T(2.0));           // minimum desired net distance.
  params->set_time_headway(T(1.0));  // desired time headway to lead vehicle.
  params->set_delta(T(4.0));  // recommended choice of free-road exponent.
  params->set_car_length(T(4.6));    // length of leading car.
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmPlanner<double>;
//template class IdmPlanner<drake::TaylorVarXd>;
//template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake

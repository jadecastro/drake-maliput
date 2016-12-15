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

// TODO(jadecastro): Do away with any/all need to depend on traffic car paths.
template <typename T>
TargetSelectorAndIdmMergePlanner<T>::TargetSelectorAndIdmMergePlanner(
    const maliput::utility::InfiniteCircuitRoad* road,
    const maliput::utility::InfiniteCircuitRoad* road_traffic,
    const int num_cars,
    const int num_targets_per_car, const bool do_restrict_to_lane,
    const bool do_sort)
    : road_(road), road_traffic_(road_traffic),
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
  //std::cerr << "  self position: " << basic_input_self->GetAtIndex(0) << "\n";

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
    //std::cerr << "  world car " << i
    //          << " position: " << basic_input_world->GetAtIndex(0) << "\n";

    inputs_world.push_back(basic_input_world);
  }

  // Obtain the output pointer.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Mimics the selection of targets (observed traffic cars).
  std::vector<CarData> car_data_targets;
  CarData car_data_self =
      SelectCarState(basic_input_self, inputs_world, &car_data_targets);

  // Invoke a planner to compute the car accelerations based on target
  // observables.
  ComputeIdmAccelerations(car_data_self, car_data_targets, context,
                          output_vector);
}

//namespace {
const double kEnormousDistance = 1e12;
const double kPerceptionDistance = 60.0;  // Targets are imperceptible
                                          // outside this radius.
const double kHorizonSeconds = 10.;
const double kNonZeroSpeed = 1e-12;
//const double kLudicruslySmallTime = 1e-12;
// TODO(jadecastro): Store these as IdmParameters or CarParameters.

template <typename T>
void TargetSelectorAndIdmMergePlanner<T>::UnwrapEndlessRoadCarState(
    const SourceState& source_states_self,
    const maliput::utility::InfiniteCircuitRoad& road,
    std::vector<PathRecord>* path_self_car) const {

  const double horizon_meters =
      source_states_self.longitudinal_speed * kHorizonSeconds;
  DRAKE_DEMAND(horizon_meters >= 0.);

  const double s_absolute_self = source_states_self.s_absolute;
  //std::cerr << "   %%% source_states_self.rp.pos.s:"
  //          << source_states_self.rp.pos.s << std::endl;
  //std::cerr << "   %%% s_absolute_self:" << s_absolute_self << std::endl;
  const double circuit_s0 = road.lane()->circuit_s(s_absolute_self);

  int path_index = road.GetPathIndex(circuit_s0);
  //std::cerr << "   %%% path_index:" << path_index << std::endl;
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
  // NB: Defaults the targets as static obstacles at infinity.
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

  // NB: Starting at the current self car lane, crawl
  // through the segments and return the data for the next car in
  // the train.
  double lane_length_sum = 0.;  // Sum of the segment lengths crawled
                                // through so far.
  bool is_first = true;
  double bound_nudge = 0.1 * params.car_length();
  // Loop over the partial path constructed over the perception
  // horizon.
  //
  // TODO(jadecastro): Most of the work of looping through the entire
  // road path is completely implementation-specific and should be
  // hidden from the user in an API.  This of course would ease
  // (hopefully seamless) portability should any of the road
  // definition change.
  while (lane_this_car != path_self_car.end()) {
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

    // NB: `states_this_car` contains states and index of the
    // next car in the train.
    bool is_car_past_limit;
    int index_this_car;
    if (!lane_this_car->is_reversed) {
      auto states_this_car =
          cars_by_lane_and_s[lane_this_car->lane].upper_bound(last_position +
                                                              bound_nudge);
      auto lane_limit = cars_by_lane_and_s[lane_this_car->lane].end();
      is_car_past_limit = states_this_car != lane_limit;
      index_this_car = states_this_car->second;
    } else {
      auto states_this_car =
          cars_by_lane_and_s[lane_this_car->lane].lower_bound(last_position -
                                                              bound_nudge);
      auto lane_limit = cars_by_lane_and_s[lane_this_car->lane].end();
      is_car_past_limit = states_this_car != lane_limit;
      index_this_car = states_this_car->second;
    }

    // TODO (jadecastro): This check is somewhat fragile; it could
    // cause false alarms and make the sim crash inadvertantly.
    if (is_car_past_limit) {
      const SourceState& this_car = source_states_targets[index_this_car];
      rp_this_car = this_car.rp;
      // TODO (jadecastro): This is somewhat of a hack.
      bool is_not_reversed_or_infty =
          !lane_this_car->is_reversed ||
          (rp_this_car.pos.s > 0.5 * kEnormousDistance);
      const double pos_relative_to_lane =
          (is_not_reversed_or_infty) ? (rp_this_car.pos.s - lane_datum)
                                      : (lane_datum - rp_this_car.pos.s);
      // Compute the relative position and velocity of the next car found.
      delta_position =
          lane_length_sum + pos_relative_to_lane - params.car_length();
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
  //std::cerr << "  @@@@@ IdmPlanner  delta_position: " << delta_position
  //          << std::endl;
  //std::cerr << "  @@@@@ IdmPlanner  delta_velocity: " << delta_velocity
  //          << std::endl;
  // Populate the relative target quantities.

  return std::make_pair(delta_position, delta_velocity);
}

//enum LaneRelation { kIntersection, kMerge, kSplit };

template <typename T>
typename TargetSelectorAndIdmMergePlanner<T>::LaneRelation
TargetSelectorAndIdmMergePlanner<T>::DetermineLaneRelation(
    const PathRecord& pra,
    const PathRecord& prb) const {
  // Assuming that the lanes described by pra and prb belong to the same
  // junction, determine how they related.

  // If they terminate at same BranchPoint, they merge.
  // Otherwise, if they originate at same BranchPoint, they split.
  // Otherwise, they merely intersect.
  // TODO(maddog)  Two other goofy cases:  split-merge, and tangent-loops.

  // TODO(jadecastro): Should go into an api.
  const maliput::api::BranchPoint* a_end_pt =
      pra.lane->GetBranchPoint(
          (pra.is_reversed) ? maliput::api::LaneEnd::kStart
          : maliput::api::LaneEnd::kFinish);
  const maliput::api::BranchPoint* b_end_pt =
      prb.lane->GetBranchPoint(
          (prb.is_reversed) ? maliput::api::LaneEnd::kStart
          : maliput::api::LaneEnd::kFinish);
  // TODO(maddog) Also ensure branches are confluent.
  if (b_end_pt == a_end_pt) { return kMerge; }

  const maliput::api::BranchPoint* a_start_pt =
      pra.lane->GetBranchPoint(
          (pra.is_reversed) ? maliput::api::LaneEnd::kFinish
          : maliput::api::LaneEnd::kStart);
  const maliput::api::BranchPoint* b_start_pt =
      prb.lane->GetBranchPoint(
          (prb.is_reversed) ? maliput::api::LaneEnd::kFinish
          : maliput::api::LaneEnd::kStart);
  // TODO(maddog) Also ensure branches are confluent.
  if (b_start_pt == a_start_pt) { return kSplit; }

  return kIntersection;
}

template <typename T>
std::tuple<double, double, double, double>
TargetSelectorAndIdmMergePlanner<T>::AssessIntersections(
    const IdmPlannerParameters<T>& params,
    const SourceState& source_states_self,
    const std::vector<SourceState>& source_states_targets,
    const std::vector<PathRecord>& path_self_car,
    const maliput::utility::InfiniteCircuitRoad& road) const {
  // TODO(jadecastro): Why am I bringing in a global???

  // Indexing Phase.
  // For each car, measure/record the 'time-in' and 'time-out' to all junctions
  // expected to be traversed within kEventHorizon.
  struct TimeBox {
    int car_index;
    PathRecord pr;
    double time_in;
    double time_out;
    double s_in;  // Distance to entry from current position.
    double s_out; // Distance to exit from current position.
  };
  std::map<const maliput::api::Junction*, TimeBox> self_box_by_junction;
  std::map<const maliput::api::Junction*,
           std::vector<TimeBox>> target_boxes_by_junction;
  std::vector<const maliput::api::Junction*> self_junctions;
  std::map<int, std::vector<const maliput::api::Junction*>>
    target_junctions_by_car;

  // Extract Junctions and TimeBoxes for the self car.
  const SourceState& self = source_states_self;
  DRAKE_DEMAND(self.rp.lane == path_self_car[0].lane);
  double lane_length_sum = 0.;
  bool is_first = true;
  for (const PathRecord& section: path_self_car) {
    double lane_length{};
    if (!section.is_reversed) {
      const double lane_datum = (is_first) ? self.rp.pos.s : 0.;
      lane_length = section.lane->length() - lane_datum;
    } else {
      const double lane_datum = (is_first) ? self.rp.pos.s :
          section.lane->length();
      lane_length = lane_datum;
    }
    DRAKE_DEMAND(lane_length > 0.);
    const double s_in = lane_length_sum;
    lane_length_sum += lane_length;
    const double s_out = lane_length_sum;
    // TODO(maddog) Decide if better way to deal with zero speed.
    const double kNonZeroSpeed = 1e-12;
    const double speed = std::max(self.longitudinal_speed, kNonZeroSpeed);
    // TODO(maddog) time in/out should account for length of vehicle, too.
    const double time_in = s_in / speed;
    const double time_out = s_out / speed;
    const TimeBox box = TimeBox{0, section,  // NB: self car_index is not used.
                                time_in, time_out,
                                s_in, s_out};

    const maliput::api::Junction* junction =
        section.lane->segment()->junction();
    self_box_by_junction[junction] = box;
    self_junctions.push_back(junction);

    is_first = false;
  }

  // For each visible target, extract a Junction and TimeBox.
  for (int i = 0; i < (int)source_states_targets.size(); ++i) {
    const SourceState& target = source_states_targets[i];
    double lane_length_sum = 0.;
    bool is_first = true;

    // First, extract the projected path taken by the target.
    //
    // TODO(jadecastro): This incorrectly depends on the self-car's
    // path.  We want to ideally get this from the current target
    // positions and their *projected* paths, if the paths are trivial
    // to find (they're determined by simple connections).  Later,
    // this should extend to all possible paths to tease out the worst
    // case scenario.
    const double horizon_meters =
      source_states_self.longitudinal_speed * kHorizonSeconds;
    DRAKE_DEMAND(horizon_meters >= 0.);

    const double s_absolute = target.s_absolute;
    const double circuit_s0 = road_traffic_->lane()->circuit_s(s_absolute);
    int path_index = road_traffic_->GetPathIndex(circuit_s0);
    //std::cerr << "   %%% path_index:" << path_index << std::endl;
    maliput::utility::InfiniteCircuitRoad::Record path_record =
      road_traffic_->path_record(path_index);
    double circuit_s_in = circuit_s0;
    std::vector<PathRecord> path;
    while (circuit_s_in <= (circuit_s0 + horizon_meters)) {
      path.push_back({path_record.lane, path_record.is_reversed});

      // TODO(maddog) Index should decrement for s_dot < 0.
      if (++path_index >= road_traffic_->num_path_records()) {
        path_index = 0;
      }
      path_record = road_traffic_->path_record(path_index);
      circuit_s_in = path_record.start_circuit_s;
      // Handle wrap-around of "circuit s" values.
      if (circuit_s_in < circuit_s0) {
        circuit_s_in += road_traffic_->cycle_length();
      }
    }

    // Next, store the Junctions and TimeBoxes for this target, based
    // on its predicted local path.
    const double s_target = target.rp.pos.s;
    if (s_target > 0.5 * kEnormousDistance) { /* We're outside sensing range */
      for (const PathRecord& section: path) {
        const TimeBox box = TimeBox{i, section,
                                    0., 0.,
                                    -kEnormousDistance, kEnormousDistance};
        const maliput::api::Junction* junction =
            section.lane->segment()->junction();
        target_boxes_by_junction[junction].push_back(box);
        target_junctions_by_car[i].push_back(junction);
      }
      continue;
    }
    for (const PathRecord& section: path) {
      double lane_length{};
      if (!section.is_reversed) {
        const double lane_datum = (is_first) ? s_target : 0.;
        lane_length = section.lane->length() - lane_datum;
      } else {
        const double lane_datum = (is_first) ? s_target :
          section.lane->length();
        lane_length = lane_datum;
      }
      DRAKE_DEMAND(lane_length > 0.);
      const double s_in = lane_length_sum;
      lane_length_sum += lane_length;
      const double s_out = lane_length_sum;
      // TODO(maddog) Decide if better way to deal with zero speed.
      const double speed = std::max(target.longitudinal_speed, kNonZeroSpeed);
      // TODO(maddog) time in/out should account for length of vehicle, too.
      const double time_in = s_in / speed;
      const double time_out = s_out / speed;
      const TimeBox box = TimeBox{i, section,
                                  time_in, time_out,
                                  s_in, s_out};

      const maliput::api::Junction* junction =
          section.lane->segment()->junction();
      target_boxes_by_junction[junction].push_back(box);
      target_junctions_by_car[i].push_back(junction);

      is_first = false;
    }
  }

  // Find cars which are entering the same junction at the same time
  // as the self car.
  // We only want to pay attention to cars coming from the right, and
  // ultimately only care about the nearest such car.
  double might_collide_at_rear = -kEnormousDistance;
  double might_collide_at_front = kEnormousDistance;
  double maybe_diff_vel_rear{};
  double maybe_diff_vel_front{};

  // Iterate over the junctions which the self car participates in.
  for (const auto& junction : self_junctions) {

    // Find the TimeBox for this car.
    const TimeBox self_box = self_box_by_junction[junction];
    DRAKE_DEMAND(self_box.time_in < self_box.time_out);

    const double kMergeHorizonSeconds = 2.;
    const double merge_horizon =
        (kMergeHorizonSeconds * source_states_self.longitudinal_speed);

    // If self is *already in* the intersection, then skip it.
    // (E.g., keep going and get out of the intersection!)
    if (self_box.time_in == 0.) { continue; }

    // Are there any intersecting TimeBoxes from other cars at this junction?
    for (const auto& other_box : target_boxes_by_junction[junction]) {
      // Skip same lane (which cannot be an intersecting path).
      if (other_box.pr.lane == self_box.pr.lane) { continue;}
      // TODO(maddog) It would probably be more efficient to keep track of
      //              "cars i,j merging at junction q" instead of rescanning
      //              here.
      // TODO(maddog) Isn't it the case, if two lanes in a junction
      //              have a branch-point in common, that they must be
      //              on the same a- or b-side of that branch-point?
      //              (I.e., they must be confluent branches of each other,
      //              not ongoing?)  This should be a checked invariant.

      switch (DetermineLaneRelation(self_box.pr, other_box.pr)) {
        case kIntersection: {
          // TODO(jadecastro): Refactor.
          // Skip other if it enters after self enters (i.e., self is first),
          // or exits before self enters (i.e., other is out of the way).
          //DRAKE_DEMAND(other_box.time_in < other_box.time_out);
          if ((other_box.time_in > self_box.time_in) ||
              (other_box.time_out < self_box.time_in)) { continue; }

          // Finally:  there could possibly be a collision at this junction,
          // so consider self's position of entry as an obstacle.
          if (self_box.s_in < might_collide_at_front) {
            might_collide_at_front = self_box.s_in;
            maybe_diff_vel_front = source_states_self.longitudinal_speed - 0.;
          } else if (self_box.s_out > might_collide_at_rear) {
            might_collide_at_rear = self_box.s_out;
            maybe_diff_vel_rear = source_states_self.longitudinal_speed - 0.;
          }
          break;
        }
        case kMerge: {
          if (self_box.s_out > merge_horizon) { continue; }

          const double delta_position_centroids =
              self_box.s_out - other_box.s_out;
          // TODO(jadecastro): Should also consider other_box.s_in so
          // as to make the car speed up to avoid collisions.
          //
          // Traffic car is 'behind' self.
          //if (delta_position_centroids < 0.) {
            // *** Decide whether to pass this info through or
            // *** implement some logic to process it.
          //continue;
          //}
          const double kTinyDistance = 0.01;
          double delta_position = (delta_position_centroids > 0.) ?
              std::max(delta_position_centroids - params.car_length(),
                       kTinyDistance) :
              std::min(delta_position_centroids + params.car_length(),
                       -kTinyDistance);

          // Report the relative states of the car immediately in
          // front and behind the self for the lane we are merging
          // into.
          if (delta_position > 0. && delta_position < might_collide_at_front) {
            might_collide_at_front = delta_position;
            maybe_diff_vel_front =
                source_states_self.longitudinal_speed -
                source_states_targets[other_box.car_index].longitudinal_speed;
          } else if (delta_position < 0. &&
                     delta_position > might_collide_at_rear) {
            might_collide_at_rear = delta_position;
            maybe_diff_vel_rear =
                source_states_self.longitudinal_speed -
                source_states_targets[other_box.car_index].longitudinal_speed;
          }

          break;
        }
        case kSplit: {
          // TODO(maddog) Handle split.
          continue;
        }
        default: { DRAKE_ABORT(); }
      }
    }
  }

  //TODO(jadecastro): Ugh, these variable names...
  //  std::pair<double, double> collision_states_front = std::make_pair(
  //    might_collide_at_front, maybe_diff_vel_front);
  //std::pair<double, double> collision_states_rear = std::make_pair(
  //    might_collide_at_rear, maybe_diff_vel_rear);
  return std::make_tuple(might_collide_at_front, maybe_diff_vel_front,
                         might_collide_at_rear, maybe_diff_vel_rear);
}

template <typename T>
typename TargetSelectorAndIdmMergePlanner<T>::CarData
TargetSelectorAndIdmMergePlanner<T>::SelectCarState(
    const systems::BasicVector<T>* input_self_car,
    // TODO(jadecastro): ^^ Bring in as a reference?
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

  DRAKE_DEMAND(num_cars_ == (int)inputs_world_car.size() + 1);
  for (int i = 0; i < num_cars_ - 1; ++i) {
    const systems::BasicVector<double>* car_state = inputs_world_car[i];
    const double long_pos_road = car_state->GetAtIndex(0);
    const double heading_road = car_state->GetAtIndex(2);
    const double vel_forward = car_state->GetAtIndex(3);
    const maliput::api::RoadPosition rp =
        road_traffic_->ProjectToSourceRoad({long_pos_road, 0., 0.}).first;
    DRAKE_DEMAND(std::cos(heading_road) >= 0.);
    DRAKE_DEMAND(vel_forward >= 0.);

    // Ignore any cars outside the perception distance.
    if (!do_restrict_to_lane_) {
      maliput::api::GeoPosition geo_pos = rp.lane->ToGeoPosition(rp.pos);
      distances.emplace_back(
          road_traffic_->RoadGeometry::Distance(geo_pos_self, geo_pos));
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

//}  // namespace
// TODO(jadecastro): Anon. namespace in TargetSelectorAndIdmPlanner too.

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

  const T& s_absolute_self = car_data_self.s;  // Road position
  const T& s_self = car_data_self.lane_s;      // Lane position
  const T& v_self = car_data_self.lane_v;      // Along-lane velocity
  const maliput::api::Lane* lane_self = car_data_self.lane;
  // Compose the inputs into the required containers.
  LanePosition lp_self(s_self, 0., 0.);
  RoadPosition rp_self(lane_self, lp_self);
  SourceState source_states_self(s_absolute_self, rp_self, v_self);

  std::vector<SourceState> source_states_targets;
  for (auto car_data : car_data_targets) {
    const T& s_absolute_target = car_data.s;
    const T& s_target = car_data.lane_s;
    const T& v_target = car_data.lane_v;
    const maliput::api::Lane* lane_target = car_data.lane;
    LanePosition lp_target(s_target, 0., 0.);
    RoadPosition rp_target(lane_target, lp_target);
    source_states_targets.emplace_back(s_absolute_target, rp_target, v_target);
  }

  // Get the local path of the self-car.
  std::vector<PathRecord> path_self_car;
  UnwrapEndlessRoadCarState(source_states_self, *road_,
                            &path_self_car);

  // Obtain the relative quantities for the nearest car ahead of the self-car.
  std::pair<double, double> sv_relative_lane = AssessLongitudinal(
      params, source_states_self, source_states_targets, path_self_car);
  const double s_rel = sv_relative_lane.first;
  const double v_rel = sv_relative_lane.second;

  std::tuple<double, double, double, double>
      sv_relative_outoflane = AssessIntersections(
          params, source_states_self, source_states_targets,
          path_self_car, *road_);
  const double s_rel_ool_front = std::get<0>(sv_relative_outoflane);
  const double v_rel_ool_front = std::get<1>(sv_relative_outoflane);
  const double s_rel_ool_rear = std::get<2>(sv_relative_outoflane);
  const double v_rel_ool_rear = std::get<3>(sv_relative_outoflane);
  if (s_rel_ool_front < kEnormousDistance) {
    std::cerr << " Compute IDM Accelerations: s_rel_ool_front: "
              << s_rel_ool_front << std::endl;
    std::cerr << "                            v_rel_ool_front: "
              << v_rel_ool_front << std::endl;
  }
  if (s_rel_ool_rear > -kEnormousDistance) {
    std::cerr << " Compute IDM Accelerations: s_rel_ool_rear: "
              << s_rel_ool_rear << std::endl;
    std::cerr << "                            v_rel_ool_rear: "
              << v_rel_ool_rear << std::endl;
  }
  // **** Use the output!! ****

  // Check that we're supplying the planner with sane parameters and
  // inputs.
  DRAKE_DEMAND(a > 0.0);
  DRAKE_DEMAND(b > 0.0);
  DRAKE_DEMAND(s_rel > car_length);

  const T s_star =
      s_0 + v_self * time_headway + v_self * v_rel / (2 * sqrt(a * b));
  const T s_star_ool_front =
      s_0 + v_self * time_headway +
      v_self * v_rel_ool_front / (2 * sqrt(a * b));
  const T s_star_ool_rear =
      s_0 + v_self * time_headway +
      v_self * v_rel_ool_rear / (2 * sqrt(a * b));
  const T accel_factor_inlane_front = pow(s_star / s_rel, 2.0);
  const T accel_factor_outoflane_front =
      pow(s_star_ool_front / s_rel_ool_front, 2.0);
  const T accel_factor_outoflane_rear =
      pow(s_star_ool_rear / s_rel_ool_rear, 2.0);

  //std::cerr << "  IdmPlanner v_ref: " << v_ref << std::endl;
  //std::cerr << "  IdmPlanner s_self: " << s_self << std::endl;
  std::cerr << "  IdmPlanner v_self: " << v_self << std::endl;
  //std::cerr << "  IdmPlanner s_rel: " << s_rel << std::endl;
  //std::cerr << "  IdmPlanner v_rel: " << v_rel << std::endl;

  std::cerr << "  IdmPlanner accel_factor_inlane_front: "
            << accel_factor_inlane_front << std::endl;
  std::cerr << "             accel_factor_outoflane_front: "
            << accel_factor_outoflane_front << std::endl;
  std::cerr << "             accel_factor_outoflane_rear: "
            << accel_factor_outoflane_rear << std::endl;

  const double longitudinal_accel = std::min(
      a * (1.0 - pow(v_self / v_ref, delta)
           - accel_factor_inlane_front
           - accel_factor_outoflane_front
           + accel_factor_outoflane_rear), 8.);
  output_vector->SetAtIndex(0,
                            longitudinal_accel);  // Longitudinal acceleration.
  output_vector->SetAtIndex(1, 0.0);       // Lateral acceleration.

  std::cerr << "  IdmPlanner accel cmd: " << output_vector->GetAtIndex(0)
            << std::endl;
  //std::cerr << "  $$$$$$$$ Selector/IdmPlanner::ComputeIdmAccelerations."
  //          << std::endl;
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

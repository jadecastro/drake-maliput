#include "target_selector.h"

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

using maliput::api::CarData;

template <typename T>
TargetSelector<T>::TargetSelector(
    const maliput::utility::InfiniteCircuitRoad* road,
    const int num_cars, const int num_targets_per_car,
    const bool do_restrict_to_lane, const bool do_sort)
    : road_(road), num_cars_(num_cars),
      num_targets_per_car_(num_targets_per_car),
      do_restrict_to_lane_(do_restrict_to_lane), do_sort_(do_sort) {
  // Fail fast if we are asking the impossible.
  DRAKE_DEMAND(num_targets_per_car <= num_cars-1);
  // Declare an input for the self car.
  this->DeclareInputPort(systems::kVectorValued, 4,
                         systems::kContinuousSampling);
  // Declare an input for N-1 remaining cars in the world.
  for (int i = 0; i < num_cars-1; ++i) {
    this->DeclareInputPort(systems::kVectorValued, 4,
                           systems::kContinuousSampling);
  }
  // Declare two output ports per car:
  //  - A vector-valued output containing the longitudinal state component.
  //  - An abstract output containing the Lane data.
  // First, declare two ports for the self car.
  //this->DeclareInputPort(systems::kVectorValued, 2,
  //                       systems::kContinuousSampling);
  this->DeclareAbstractOutputPort(systems::kContinuousSampling);
  // Next, declare two ports for each of the M target traffic cars of interest.
  for (int i = 0; i < num_targets_per_car; ++i) {
    //this->DeclareInputPort(systems::kVectorValued, 2,
    //                     systems::kContinuousSampling);
    this->DeclareAbstractOutputPort(systems::kContinuousSampling);
  }
}

template <typename T>
TargetSelector<T>::~TargetSelector() {}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelector<T>::get_self_inport() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelector<T>::get_world_inport(const int i) const {
  DRAKE_DEMAND(i < num_cars_);
  return systems::System<T>::get_input_port(i+1);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelector<T>::get_self_outport() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
TargetSelector<T>::get_target_outport(const int i) const {
  DRAKE_DEMAND(i < num_targets_per_car_);
  return systems::System<T>::get_output_port(i+1);
}

template <typename T>
void TargetSelector<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  std::cerr << " ^^^^^^^^ TargetSelector::EvalOutput ..." << std::endl;
  // Obtain the self-car input.
  const systems::BasicVector<T>* basic_input_self =
      this->EvalVectorInput(context, this->get_self_inport().get_index());
  //std::cerr << "TargetSelector EvalVectorInput...\n";
  DRAKE_ASSERT(basic_input_self);
  //const EndlessRoadCarState<T>* const input_self_car =
  //  dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_self);
  //DRAKE_ASSERT(input_self_car);
  std::cerr << "  self position: " << basic_input_self->GetAtIndex(0) << "\n";

  // Obtain the world-car inputs.
  //std::vector<const EndlessRoadCarState<T>*> inputs_world;
  std::vector<const systems::BasicVector<T>*> inputs_world;
  for (int i = 0; i < num_cars_-1; ++i) {
    std::cerr << "TargetSelector EvalVectorInput..." << std::endl;
    const systems::BasicVector<T>* basic_input_world =
      this->EvalVectorInput(context, this->get_world_inport(i).get_index());
    std::cerr << "TargetSelector EvalVectorInput." << std::endl;
    DRAKE_ASSERT(basic_input_world);
    //const EndlessRoadCarState<T>* const input_world =
    //  dynamic_cast<const EndlessRoadCarState<T>*>(basic_input_world);
    //DRAKE_ASSERT(input_world);
    //inputs_world.push_back(input_world);
    std::cerr << "  world car " << i<< " position: " <<
        basic_input_world->GetAtIndex(0) << "\n";

    inputs_world.push_back(basic_input_world);
  }

  // Obtain the output pointers.
  std::cerr << "TargetSelector GetMutableData (self)..." << std::endl;
  systems::AbstractValue* const output_data_self =
    output->GetMutableData(0);
  CarData& output_self = output_data_self->GetMutableValue<CarData>();
  std::cerr << "TargetSelector GetMutableData (self)." << std::endl;

  std::cerr << "TargetSelector num_targets_per_car_:" <<
      num_targets_per_car_ << std::endl;
  std::vector<CarData*> outputs_target;
  for (int i = 0; i < num_targets_per_car_; ++i) {
    std::cerr << "TargetSelector GetMutableValue (target)..." << std::endl;
    systems::AbstractValue* const output_data_target =
        output->GetMutableData(i+1);
    std::cerr << "TargetSelector GetMutableValue 2..." << std::endl;
    CarData& output_value_target =
        output_data_target->GetMutableValue<CarData>();
    outputs_target.emplace_back(output_value_target);
    std::cerr << "TargetSelector GetMutableValue (target)." << std::endl;
  }

  //std::cerr << "TargetSelector EvalVectorInput 3...\n";
  SelectCarStateAndEvalOutput(basic_input_self, inputs_world,
                              &output_self, outputs_target);
  std::cerr << " ^^^^^^^^ TargetSelector::EvalOutput." << std::endl;
}

const double kEnormousDistance = 1e12;
const double kPerceptionDistance = 60.0;  // Targets are imperceptible
                                          // outside this radius.
// TODO(jadecastro): Store these as IdmParameters or CarParameters.

template <typename T>
void TargetSelector<T>::SelectCarStateAndEvalOutput(
    const systems::BasicVector<T>* input_self_car,
    const std::vector<const systems::BasicVector<T>*>& inputs_world_car,
    CarData* output_self,
    std::vector<CarData*> outputs_target) const {

  maliput::api::GeoPosition geo_pos_self;
  std::vector<double> distances;
  std::vector<CarData> car_data;
  std::pair<T,T> pair;
  DRAKE_DEMAND(num_cars_ == (int) inputs_world_car.size()+1);
  for (int i = 0; i < num_cars_; ++i) {
    //std::cerr << "TargetSelector::UnwrapEndlessRoadCarState...\n";
    //std::cerr << "     i: " << i << "\n";
    //std::cerr << "     num world cars: " << inputs_world_car.size() << "\n";
    const systems::BasicVector<double>* car_state =
      (i == 0) ? input_self_car : inputs_world_car[i-1];
    std::cerr << "  position: " << car_state->GetAtIndex(0) << "\n";
    const maliput::api::RoadPosition rp = road_->ProjectToSourceRoad(
        {car_state->GetAtIndex(0), 0., 0.}).first;
    maliput::api::Lane rp_lane = rp.lane;
    double rp_pos = rp.pos.s;

    //std::cerr << "TargetSelector::UnwrapEndlessRoadCarState 0...\n";
    DRAKE_DEMAND(std::cos(car_state->GetAtIndex(2)) >= 0.);
    DRAKE_DEMAND(car_state->GetAtIndex(3) >= 0.);

    // Ignore any cars outside the perception distance.
    if (!do_restrict_to_lane_) {
      maliput::api::GeoPosition geo_pos = rp.lane->ToGeoPosition(rp.pos);
      if (i == 0) {
        geo_pos_self = geo_pos;
        double position_enormous = kEnormousDistance;
        // Seed the output with "infinite" obstacles.  TODO
        // (jadecastro): Is the lane currently occupied by the
        // self-car the best proxy to use here?
        //std::pair<T,T> pair = std::make_pair(T{kEnormousDistance}, T{0.});
        CarData car_data_infinite(position_enormous, 0., rp_lane);
        for (int i = 0; i < (int) outputs_target.size(); ++i) {
          outputs_target[i] = &car_data_infinite;
        }

        // Populate the output for the self car.
        const double longitudinal_speed =  // Along-lane speed.
            car_state->GetAtIndex(3) * std::cos(car_state->GetAtIndex(2));
        //pair = std::make_pair(T{rp.pos.s}, T{longitudinal_speed});
        output_self->set(rp_pos, longitudinal_speed, rp_lane);
        //output_self = car_data_self;
      } else {
        distances.emplace_back(road_->
                               RoadGeometry::Distance(geo_pos_self, geo_pos));
      }
    } else {
      DRAKE_ABORT();
      // TODO(jadecastro): Fill me in......
    }

   if (i > 0 && distances[i] < kPerceptionDistance) {
     const double longitudinal_speed =  // Along-lane speed.
         car_state->GetAtIndex(3) * std::cos(car_state->GetAtIndex(2));
     //pair = std::make_pair(T{rp.pos.s}, T{longitudinal_speed});
     //car_data.emplace_back(std::make_pair(&pair, rp.lane));
     CarData car_data_value(rp_pos, longitudinal_speed, rp_lane);
     car_data.emplace_back(car_data_value);
    }
  }

  // Populate the output ports with data for each of the registered targets.
  std::vector<int>
      index_set(distances.size());  // TODO(jadecastro): Must be a better way.
  for (int i = 0; i < (int) distances.size(); ++i) { index_set[i] = i; }
  const std::vector<int> possibly_sorted_distances = (do_sort_) ?
      SortDistances(distances) :
      index_set;
  for (auto i : possibly_sorted_distances) {
    if (i >= num_targets_per_car_) { break; }
    outputs_target[i] = &car_data[i];
  }
}

// Lambda-expression approach to sort indices (adapted from
// http://stackoverflow.com/questions/1577475/ ...
//   c-sorting-and-keeping-track-of-indexes)
template <typename T>
std::vector<int> TargetSelector<T>::SortDistances(const std::vector<T>& v)
    const {
  // initialize original index locations
  std::vector<int> indices(v.size());
  std::iota(indices.begin(), indices.end(), 0);
  sort(indices.begin(), indices.end(),
       [&v](int i1, int i2) {return v[i1] < v[i2];});
  return indices;
}
  /*
template <typename T>
std::unique_ptr<systems::SystemOutput<T>> TargetSelector<T>::AllocateOutput(
    const systems::Context<T>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<T>> output(
    new systems::LeafSystemOutput<T>);

  // Add a port for the self car.
  CarData results_self;
  output->add_port(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<CarData>(results_self)));

  // Add a port for each of the targets.
  for (int i = 0; i < num_targets_per_car_; ++i) {
    CarData results_target;
    output->add_port(std::unique_ptr<systems::AbstractValue>(
        new systems::Value<CarData>(results_target)));
  }
  return std::move(output);
}
*/
/*
template <typename T>
std::unique_ptr<systems::SystemOutput<T>> TargetSelector<T>::AllocateOutput(
    const systems::Context<T>& context) const {
  std::unique_ptr<systems::LeafSystemOutput<T>> output =
    std::make_unique<systems::LeafSystemOutput<T>>();

  // Add a port for the self car.
  auto results_self = std::make_unique<systems::Value<CarData>>(
      CarData());
  output->add_port(std::move(results_self));

  // Add a port for each of the targets.
  for (int i = 0; i < num_targets_per_car_; ++i) {
    auto results_target = std::make_unique<systems::Value<CarData>>(
        CarData());
    output->add_port(std::move(results_target));
  }
  return std::move(output);
}
*/

// These instantiations must match the API documentation in
// target_selector.h.
// TODO(jadecastro): AutoDiff, symbolic_expression
template class TargetSelector<double>;

}  // namespace automotive
}  // namespace drake

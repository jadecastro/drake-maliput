#include "endless_road_traffic_car.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadTrafficCar<T>::EndlessRoadTrafficCar(
    const std::string& id,
    const int num_cars,
    const maliput::utility::InfiniteCircuitRoad* road,
    const T& s_init, const T& r_init, const T& v_init, const T& heading_init,
    const T& v_ref)
    : id_(id), num_cars_(num_cars) {
  /*
  for (int i = 0; i < num_cars - 1; ++i) {
    // Expect to mate each input port with a port in `target_inports`
    // of TargetSelector an appropriately-sized state vector (4 states).
    this->DeclareInputPort(systems::kVectorValued,
                           EndlessRoadCarStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
  }
  // Expect to retrieve each of the states for this particular (self) car.
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
  */

  // The reference velocity must be strictly positive.
  DRAKE_DEMAND(v_ref > 0);
  const int num_targets_per_car = num_cars - 1;

  systems::DiagramBuilder<T> builder;

  const TargetSelector<T>* target_selector = builder.AddSystem(
      std::make_unique<TargetSelector<T>>(road,
                                          num_cars_,
                                          num_targets_per_car));
  // TODO (jadecastro): Default num_targets_per_car as num_cars-1 if
  // no argument.

  std::cerr << "   EndlessRoadTrafficCar s_init: " << s_init << " \n";
  // Instantiate EndlessRoadSimpleCar systems at some initial state.
  car_ = builder.AddSystem(
           std::make_unique<EndlessRoadSimpleCar<T>>(road,
                                                     s_init,
                                                     r_init,
                                                     v_init,
                                                     heading_init));

  // Instantiate an IdmPlanner to feed the car its input.
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
      road,
      v_ref, /* desired velocity */
      num_targets_per_car));

  std::cerr << "... Attempting to connect EndlessRoadEgoCar.\n";
  std::cerr << "       target_selector->get_num_output_ports:" <<
    target_selector->get_num_output_ports() << std::endl;
  std::cerr << "       num_targets_per_car:" <<
    num_targets_per_car << std::endl;
  DRAKE_DEMAND(target_selector->get_num_output_ports() ==
               num_targets_per_car+1);
  DRAKE_DEMAND(planner_->get_num_input_ports() == num_targets_per_car+1);
  builder.Connect(target_selector->get_self_outport(),
                  planner_->get_self_inport());
  std::cerr << "TargetSelector connected to Planner (self port).\n";
  for (int i = 0; i < num_targets_per_car; ++i) {
    builder.Connect(target_selector->get_target_outport(i),
                    planner_->get_target_inport(i));
  }
  std::cerr << "TargetSelector connected to Planner (target ports).\n";
  builder.Connect(*planner_, *car_);
  std::cerr << "Planner connected to Car.\n";
  builder.Connect(car_->get_state_output_port(),
                  target_selector->get_self_inport());
  std::cerr << "Car connected to TargetSelector.\n";

  // Require N-1 unsorted input ports of the world cars to TargetSelector.
  std::cerr << "Exporting " << num_cars_-1 << " input ports.\n";
  for (int i = 0; i < num_cars_-1; ++i) {
    builder.ExportInput(target_selector->get_world_inport(i));
  }
  std::cerr << "Exporting the output port.\n";
  builder.ExportOutput(car_->get_state_output_port());  // Exports the
                                                        // car states
                                                        // to port 0.

  builder.BuildInto(this);
}

template <typename T>
void EndlessRoadTrafficCar<T>::SetDefaultState(
    systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for the car.
  DRAKE_DEMAND(context != nullptr);
  systems::Context<T>* context_car =
      this->GetMutableSubsystemContext(context, car_);
  DRAKE_DEMAND(context_car != nullptr);

  const double& thing = car_->get_s_init();
  std::cerr << "  EndlessRoadTrafficCar s_init: " << thing << ".\n";

  // Set the default state based on the member fields.
  car_->SetDefaultState(context_car);
}

// These instantiations must match the API documentation in
// endless_road_traffic_car.h.
// TODO(jadecastro): Adopt a wider array of scalar types.
template class EndlessRoadTrafficCar<double>;

}  // namespace automotive
}  // namespace drake

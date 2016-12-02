#include "endless_road_ego_car.h"

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
EndlessRoadEgoCar<T>::EndlessRoadEgoCar(
    const std::string& id,
    const int num_cars,
    const maliput::utility::InfiniteCircuitRoad* road,
    const T& s_init, const T& r_init, const T& v_init, const T& heading_init,
    const T& v_ref)
    : v_ref_(v_ref), id_(id), num_cars_(num_cars) {
  /*
  for (int i = 0; i < num_cars - 1; ++i) {
    // Expect to mate each input port with a port in `target_inports`
    // of DecisionLayer an appropriately-sized state vector (4 states).
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
  v_ref_ = 10.;
  DRAKE_DEMAND(v_ref > 0);

  systems::DiagramBuilder<T> builder;

  const DecisionLayer<T>* decision_layer = builder.AddSystem(
      std::make_unique<DecisionLayer<T>>(road,
                                         num_cars_,
                                         1 /* IDM expects one target */));

  std::cerr << "   EndlessRoadEgoCar s_init: " << s_init << " \n";
  // Instantiate EndlessRoadSimpleCar systems at some initial state.
  car_ = builder.AddSystem(
           std::make_unique<EndlessRoadSimpleCar<T>>(road,
                                                     s_init,
                                                     r_init,
                                                     v_init,
                                                     heading_init));

  // Instantiate an IdmPlanner to feed the car its input.
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
      v_ref_ /* desired velocity */));

  std::cerr << "... Attempting to connect EndlessRoadEgoCar.\n";
  DRAKE_DEMAND(decision_layer->get_num_output_ports() == 1);
  builder.Connect(decision_layer->get_output_port(),
                  planner_->get_target_port());
  std::cerr << "DecisionLayer connected to Planner.\n";
  builder.Connect(*planner_, *car_);
  std::cerr << "Planner connected to Car.\n";
  builder.Connect(car_->get_s_axis_output_port(), planner_->get_ego_port());
  std::cerr << "Car connected to Planner.\n";
  builder.Connect(car_->get_state_output_port(),
                  decision_layer->get_self_input_port());
  std::cerr << "Car connected to DecisionLayer.\n";

  // Require N-1 unsorted input ports of the world cars to DecisionLayer.
  std::cerr << "Exporting " << num_cars_-1 << " input ports.\n";
  for (int i = 0; i < num_cars_-1; ++i) {
    builder.ExportInput(decision_layer->get_world_input_port(i));
  }
  std::cerr << "Exporting the output port.\n";
  builder.ExportOutput(car_->get_state_output_port());  // Exports the
                                                        // car states
                                                        // to port 0.

  builder.BuildInto(this);
}

template <typename T>
void EndlessRoadEgoCar<T>::SetDefaultState(
    systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for the car.
  DRAKE_DEMAND(context != nullptr);
  systems::Context<T>* context_car =
      this->GetMutableSubsystemContext(context, car_);
  DRAKE_DEMAND(context_car != nullptr);

  const double& thing = car_->get_s_init();
  std::cerr << "  EndlessRoadEgoCar s_init: " << thing << ".\n";

  // Set the default state based on the member fields.
  car_->SetDefaultState(context_car);
}

// These instantiations must match the API documentation in
// endless_road_traffic_car.h.
// TODO(jadecastro): Adopt a wider array of scalar types.
template class EndlessRoadEgoCar<double>;

}  // namespace automotive
}  // namespace drake

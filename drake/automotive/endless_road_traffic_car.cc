#include "endless_road_traffic_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadTrafficCar<T>::EndlessRoadTrafficCar(
    const std::string& id,
    const maliput::utility::InfiniteCircuitRoad* road,
    const T& x_init, const T& v_init,
    const T& v_ref)
    : id_(id)  {
  this->DeclareInputPort(systems::kVectorValued,
                         EndlessRoadOracleOutputIndices::kNumCoordinates,
                         systems::kContinuousSampling);

  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);

  // The reference velocity must be strictly positive.
  DRAKE_DEMAND(v_ref > 0);

  systems::DiagramBuilder<T> builder;

  // Instantiate EndlessRoadSimpleCar systems at some initial state.
  car_ = builder.AddSystem(
      std::make_unique<EndlessRoadSimpleCar<T>>(x_init, v_init, road));

  // Instantiate additional subsystems feed the two cars their inputs.
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
      v_ref /* desired velocity of the ego car */));

  builder.Connect(*planner_, *car_);
  builder.Connect(car_->get_output_port(), planner_->get_ego_port());

  builder.ExportOutput(car_->get_output_port());  // Exports to port 0.

  builder.BuildInto(this);
}

template <typename T>
void EndlessRoadTrafficCar<T>::SetDefaultState(
    systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for each car.
  DRAKE_DEMAND(context != nullptr);
  systems::Context<T>* context_car =
      this->GetMutableSubsystemContext(context, car_);
  DRAKE_DEMAND(context_car != nullptr);

  // Set the default state.
  car_->SetDefaultState(context_car);
}

}  // namespace automotive
}  // namespace drake

#pragma once

#include <memory>

#include "drake/automotive/endless_road_simple_car.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/target_selector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}  // namespace utility
}  // namespace maliput
namespace automotive {

/// System consisting of a car and planner.
///
/// TODO(jadecastro): Add in a description of the TargetSelector.
///
///  sigma_target,
///  sigma_dot_target
///       |
///       | port
///       |  1   +--------------+         +-------------+
///       +----->|   Planner    | v_dot_e |   Ego Car   |  x_e, v_e
///              |              |-------->|             |----+
///       +----->| (IdmPlanner) |         | (LinearCar) |    |
///       | port +--------------+         +-------------+    |
///       |  0                                               |
///       +--------------------------------------------------+
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
template <typename T>
class EndlessRoadEgoCar : public systems::Diagram<T> {
 public:
  /// Constructs a two-car system.
  ///
  /// @p v_ref desired velocity of the ego (controlled) car.
  /// @p a_agent constant acceleration of the agent car.
  EndlessRoadEgoCar(const std::string& id, const int num_cars,
                    const maliput::utility::InfiniteCircuitRoad* road,
                    const T& s_init, const T& r_init, const T& v_init,
                    const T& heading_init, const T& v_ref);

  ~EndlessRoadEgoCar() override {}

  /// Gets the car id.
  const std::string get_id() const { return id_; }

  /// Sets the continuous states in @p context to default values.
  void SetDefaultState(systems::Context<T>* context) const;

  /// Getters for the subsystems.
  const EndlessRoadSimpleCar<T>* car_system() const { return car_; }
  const IdmPlanner<T>* get_planner_system() const { return planner_; }

  // Disable copy and assignment.
  EndlessRoadEgoCar(const EndlessRoadEgoCar<T>&) = delete;
  EndlessRoadEgoCar& operator=(const EndlessRoadEgoCar<T>&) = delete;
  EndlessRoadEgoCar(EndlessRoadEgoCar<T>&&) = delete;
  EndlessRoadEgoCar& operator=(EndlessRoadEgoCar<T>&&) = delete;

 private:
  double v_ref_;
  const std::string id_;
  const int num_cars_;
  const EndlessRoadSimpleCar<T>* car_ = nullptr;
  const IdmPlanner<T>* planner_ = nullptr;
};

}  // namespace automotive
}  // namespace drake

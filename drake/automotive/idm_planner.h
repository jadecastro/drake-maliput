#pragma once

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/maliput/api/car_data.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Model) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   0: @p x_self self car position (scalar) [m]
///   1: @p v_self self car velocity (scalar) [m/s]
///   2: @p x_target relative position wrt target (scalar) [m]
///   3: @p v_target relative closing velocity wrt target (scalar) [m/s]
/// Outputs:
///   0: @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
  // TODO (jadecastro): Revise this description!
template <typename T>
class IdmPlanner : public systems::LeafSystem<T> {
 public:
  /// @p v_ref desired velocity of the ego car in units of m/s.
  explicit IdmPlanner(const maliput::utility::InfiniteCircuitRoad* road,
                      const T& v_ref, const int num_targets_per_car);
  ~IdmPlanner() override;

  /// Returns the port to the input subvector collecting ego car states.
  const systems::SystemPortDescriptor<T>& get_self_inport() const;

  /// Returns the port to the input subvector collecting relative target states.
  const systems::SystemPortDescriptor<T>& get_target_inport(const int i) const;

  // System<T> overrides.
  // The output of this system is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  // Disable copy and assignment.
  IdmPlanner(const IdmPlanner<T>&) = delete;
  IdmPlanner& operator=(const IdmPlanner<T>&) = delete;
  IdmPlanner(IdmPlanner<T>&&) = delete;
  IdmPlanner& operator=(IdmPlanner<T>&&) = delete;

 protected:
  struct SourceState {
    SourceState() {}

  SourceState(maliput::api::RoadPosition _rp, double _ls)
  : rp(_rp), longitudinal_speed(_ls) {}

    maliput::api::RoadPosition rp;
    double longitudinal_speed{};
  };

  struct PathRecord {
    const maliput::api::Lane* lane{};
    bool is_reversed{};
  };

private:
  //typedef std::pair<std::pair<T,T>*,
  //    const maliput::api::Lane*> CarData;

  void UnwrapEndlessRoadCarState(
      const SourceState& source_state_self,
      const maliput::utility::InfiniteCircuitRoad& road,
      std::vector<PathRecord>* path_self_car) const;

  std::pair<double, double> AssessLongitudinal(
            const IdmPlannerParameters<T>& params,
            const SourceState& source_states_self,
            const std::vector<SourceState>& source_states_target,
            const std::vector<PathRecord>& path_self_car) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const T v_ref_;  // Desired velocity.
  const int num_targets_per_car_;
};

}  // namespace automotive
}  // namespace drake

#pragma once

#include <stdexcept>

#include <boost/optional.hpp>

#include "drake/automotive/gen/idm_planner_parameters.h"
//#include "drake/automotive/maliput/api/car_data.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

// TODO(jadecastro): Need?
namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}  // namespace utility
}  // namespace maliput

namespace automotive {
/// A decision module that acts like a fake perception layer.  Its
/// sole purpose is to decide which cars are perceived by a certain
/// ego car.  Given @p N-1 cars in the world, TargetSelector outputs a
/// signal to a planner ensuring collision avoiance with @p M cars of
/// immediate interest to the ego.
///
/// (Elevation-above-road 'h' is implicitly zero, too.)
///
/// state vector
/// * planar LANE-space position:  s, r
/// * planar isometric LANE-space velocity:  (sigma, rho)-dot
///
/// input vector:
/// * Currently:  none (accelerations are just zero)
/// * Later:  planar isometric LANE-space acceleration: (sigma, rho)-ddot
///
/// output vector: same as state vector
// TODO (jadecastro): Revise this description!
template <typename T>
class TargetSelectorAndIdmMergePlanner : public systems::LeafSystem<T> {
 public:
  explicit TargetSelectorAndIdmMergePlanner(
      const maliput::utility::InfiniteCircuitRoad* road, const int num_cars,
      const int num_targets_per_car, const bool do_restrict_to_lane = false,
      const bool do_sort = true);
  ~TargetSelectorAndIdmMergePlanner() override;

  /// Returns the port to the input collecting states for the 'self' car.
  const systems::SystemPortDescriptor<T>& get_self_inport() const;

  /// Returns the port to the input collecting states for world car i.
  const systems::SystemPortDescriptor<T>& get_world_inport(const int i) const;

  // System<T> overrides.
  // The output of TargetSelectorAndIdmMergePlanner is an algebraic
  // relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  // TODO (jadecastro): Disable copy and assignemnt.

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

 protected:
  struct CarData {
    CarData(const double _s, const double _lane_s, const double _lane_v,
            const maliput::api::Lane* _lane)
       : s(_s), lane_s(_lane_s), lane_v(_lane_v), lane(_lane) {}

    const double s;
    const double lane_s;
    const double lane_v;
    const maliput::api::Lane* lane;
  };
  // LeafSystem<T> overrides
  // std::unique_ptr<systems::SystemOutput<T>> AllocateOutput(
  //    const systems::Context<T>& context) const override;

 private:
  struct SourceState {
    SourceState() {}

    SourceState(maliput::api::RoadPosition arp, double als)
        : rp(arp), longitudinal_speed(als) {}

    maliput::api::RoadPosition rp;
    double longitudinal_speed{};
  };

  struct PathRecord {
    const maliput::api::Lane* lane{};
    bool is_reversed{};
  };

  TargetSelectorAndIdmMergePlanner::CarData SelectCarState(
      const systems::BasicVector<T>* input_self_car,
      const std::vector<const systems::BasicVector<T>*>& inputs_world_car,
      std::vector<CarData>* car_data_targets) const;

  void UnwrapEndlessRoadCarState(
      const SourceState& source_state_self,
      const double& s_absolute,
      const maliput::utility::InfiniteCircuitRoad& road,
      std::vector<PathRecord>* path_self_car) const;

  std::pair<double, double> AssessLongitudinal(
      const IdmPlannerParameters<T>& params,
      const SourceState& source_states_self,
      const std::vector<SourceState>& source_states_target,
      const std::vector<PathRecord>& path_self_car) const;

  void ComputeIdmAccelerations(const CarData& car_data_self,
                               const std::vector<CarData>& car_data_target,
                               const systems::Context<T>& context,
                               systems::BasicVector<T>* output_vector) const;

  std::vector<int> SortDistances(const std::vector<T>& v) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const int num_cars_;
  const int num_targets_per_car_;
  const bool do_restrict_to_lane_;
  const bool do_sort_;
};

}  // namespace automotive
}  // namespace drake

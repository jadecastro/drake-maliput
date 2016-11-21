#pragma once

#include <stdexcept>

#include <boost/optional.hpp>

#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}  // namespace utility
}  // namespace maliput

namespace automotive {

/// TODO(jadecastro): Rename this to SimulatedPerceptionLayer or similar.

/// A decision module that acts like a fake perception layer.  Its
/// sole purpose is to decide which cars are perceived by a certain
/// ego car.  Given @p N-1 cars in the world, DecisionLayer outputs a
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
template <typename T>
class DecisionLayer : public systems::LeafSystem<T> {
 public:
  explicit DecisionLayer(const maliput::utility::InfiniteCircuitRoad* road,
                const int num_cars,
                const int num_targets_per_car);
  ~DecisionLayer() override;

  /// Returns the port to the input collecting states for the 'self' car.
  const systems::SystemPortDescriptor<T>& get_self_input_port() const;

  /// Returns the port to the input collecting states for world car i.
  const systems::SystemPortDescriptor<T>&
      get_world_input_port(const int i) const;

  // System<T> overrides.
  // The output of DecisionLayer is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

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

  void DoEvalOutput(
    const EndlessRoadCarState<T>* self_car_input,
    const std::vector<const EndlessRoadCarState<T>*>& world_car_inputs,
    std::vector<EndlessRoadOracleOutput<T>*>& target_outputs) const;

  void UnwrapEndlessRoadCarState(
    const EndlessRoadCarState<double>* self_car_input,
    const std::vector<const EndlessRoadCarState<double>*>& world_car_inputs,
    const maliput::utility::InfiniteCircuitRoad* road,
    const double horizon_seconds,
    SourceState* self_source_state,
    std::vector<SourceState>* world_source_states,
    std::vector<PathRecord>* self_car_path) const;

  void AssessLongitudinal(
    const SourceState& self_source_states,
    const std::vector<SourceState>& world_source_states,
    std::vector<PathRecord>& self_car_path,
    std::vector<EndlessRoadOracleOutput<double>*>& target_outputs) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const int num_cars_;
  const int num_targets_per_car_;
  // TODO(maddog)  Do we need to keep track of these here?
  //systems::SystemPortDescriptor<T> self_inport_;
  // TODO(jadecastro): Remove these.
  std::vector<systems::SystemPortDescriptor<T>> target_inports_;
  std::vector<systems::SystemPortDescriptor<T>> outports_;

};

}  // namespace automotive
}  // namespace drake

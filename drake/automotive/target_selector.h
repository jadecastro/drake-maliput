#pragma once

#include <stdexcept>

#include <boost/optional.hpp>

#include "drake/automotive/maliput/api/car_data.h"
//#include "drake/automotive/maliput/api/lane_data.h"
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
class TargetSelector : public systems::LeafSystem<T> {
 public:
  explicit TargetSelector(const maliput::utility::InfiniteCircuitRoad* road,
                          const int num_cars,
                          const int num_targets_per_car,
                          const bool do_restrict_to_lane = false,
                          const bool do_sort = true);
  ~TargetSelector() override;

  /// Returns the port to the input collecting states for the 'self' car.
  const systems::SystemPortDescriptor<T>& get_self_inport() const;

  /// Returns the port to the input collecting states for world car i.
  const systems::SystemPortDescriptor<T>& get_world_inport(const int i) const;

  /// Returns the output port for the self car.
  const systems::SystemPortDescriptor<T>& get_self_outport() const;

  /// Returns the output port for target car i.
  const systems::SystemPortDescriptor<T>& get_target_outport(const int i) const;

  // System<T> overrides.
  // The output of TargetSelector is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  std::unique_ptr<systems::SystemOutput<T>> AllocateOutput(
    const systems::Context<T>& context) const override;

 protected:
  // LeafSystem<T> overrides
  //std::unique_ptr<systems::SystemOutput<T>> AllocateOutput(
  //    const systems::Context<T>& context) const override;

 private:
  //typedef std::pair<std::pair<T,T>*,
  //    const maliput::api::Lane*> CarData;

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

  void SelectCarStateAndEvalOutput(
    const systems::BasicVector<T>* input_self_car,
    const std::vector<
      const systems::BasicVector<T>*>& inputs_world_car,
    maliput::api::CarData* output_self,
    std::vector<maliput::api::CarData*> outputs_target) const;

  // TODO(jadecastro): const?
  std::vector<int> SortDistances(const std::vector<T>& v) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const int num_cars_;
  const int num_targets_per_car_;
  const bool do_restrict_to_lane_;
  const bool do_sort_;
  // TODO(maddog)  Do we need to keep track of these here?
  //systems::SystemPortDescriptor<T> self_inport_;
  // TODO(jadecastro): Remove these.
  //std::vector<systems::SystemPortDescriptor<T>> target_inports_;
  //std::vector<systems::SystemPortDescriptor<T>> outports_;

};

}  // namespace automotive
}  // namespace drake

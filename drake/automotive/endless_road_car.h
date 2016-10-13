#pragma once

#include <stdexcept>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/simple_car_config.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/framework/leaf_system.h"

namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}
}

namespace drake {
namespace automotive {



/// A non-physical car that operates in LANE-space on an InfiniteCircuitRoad,
/// i.e., a maliput road network that only has a single Lane and infinite
/// longitudinal extent.
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
class EndlessRoadCar : public systems::LeafSystem<T> {
 public:
  EndlessRoadCar(const maliput::utility::InfiniteCircuitRoad* road,
                 bool ignore_input_jackass,
                 const SimpleCarConfig<T>& config = get_default_config());

  static SimpleCarConfig<T> get_default_config();

  const SimpleCarConfig<T>& config() const { return config_; }

 public:
  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>
                  > AllocateContinuousState() const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  void DoEvalOutput(const EndlessRoadCarState<T>&,
                    EndlessRoadCarState<T>*) const;

  void DoEvalTimeDerivatives(const EndlessRoadCarState<T>&,
                             const DrivingCommand<T>&,
                             EndlessRoadCarState<T>*) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const SimpleCarConfig<T> config_;
  const bool ignore_input_jackass_;
};

}  // namespace automotive
}  // namespace drake

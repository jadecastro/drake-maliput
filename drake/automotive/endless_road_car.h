#pragma once

#include <stdexcept>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/automotive/gen/simple_car_config.h"
#include "drake/common/drake_export.h"
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
  enum ControlType {
    kNone,  // no controls, i.e., constant velocity
    kUser,  // processes DrivingCommand input
    kIdm,   // IDM controller using input from EndlessRoadOracle
  };

  EndlessRoadCar(const std::string& id,
                 const maliput::utility::InfiniteCircuitRoad* road,
                 const ControlType control_type,
                 const SimpleCarConfig<T>& config = get_default_config());

  ControlType control_type() const { return control_type_; }

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
  struct Accelerations {
    Accelerations(T _forward, T _lateral)
        : forward(_forward), lateral(_lateral) {}

    T forward;
    T lateral;
  };

  void DoEvalOutput(const EndlessRoadCarState<T>&,
                    EndlessRoadCarState<T>*) const;

  Accelerations ComputeUserAccelerations(
      const EndlessRoadCarState<T>& state,
      const DrivingCommand<T>& input) const;

  Accelerations ComputeIdmAccelerations(
      const EndlessRoadCarState<T>& state,
      const EndlessRoadOracleOutput<T>& input) const;


  void DoEvalTimeDerivatives(const EndlessRoadCarState<T>&,
                             const Accelerations& accelerations,
                             EndlessRoadCarState<T>*) const;

  const std::string id_;
  const maliput::utility::InfiniteCircuitRoad* road_;
  const ControlType control_type_;
  const SimpleCarConfig<T> config_;
};

}  // namespace automotive
}  // namespace drake
#pragma once

#include <stdexcept>

#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/automotive/gen/simple_car_config.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {

namespace maliput {
namespace utility {
class InfiniteCircuitRoad;
}  // namespace utility
}  // namespace maliput

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
class EndlessRoadSimpleCar : public systems::LeafSystem<T> {
 public:

  EndlessRoadSimpleCar(const maliput::utility::InfiniteCircuitRoad* road,
                       const T& s_init, const T& r_init,
                       const T& v_init, const T& heading_init);

  /// Returns the state output port.
  const systems::SystemPortDescriptor<T>& get_state_output_port() const;

  /// Returns the s-axis output port.
  const systems::SystemPortDescriptor<T>& get_s_axis_output_port() const;

  const T& get_s_init() const { return s_init_; }

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  /// Sets the continuous states in @p context to default values.
  void SetDefaultState(systems::Context<T>* context) const;

  /*
 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>
                  > AllocateContinuousState() const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;
  */

 private:
  struct Accelerations {
    Accelerations(T _forward, T _lateral)
        : forward(_forward), lateral(_lateral) {}

    T forward;
    T lateral;
  };

  void DoEvalOutput(const EndlessRoadCarState<T>&,
                    EndlessRoadCarState<T>*) const;

  // TODO(jadecastro): This is a complete mess.
  void DoEvalTimeDerivatives(const systems::VectorBase<T>&,
                             const Accelerations& accelerations,
                             systems::VectorBase<T>*) const;

  void DoEvalTimeDerivatives(const EndlessRoadCarState<T>&,
                             const Accelerations& accelerations,
                             EndlessRoadCarState<T>*) const;

  const maliput::utility::InfiniteCircuitRoad* road_;
  const T& s_init_;
  const T& r_init_;
  const T& v_init_;
  const T& heading_init_;
};

}  // namespace automotive
}  // namespace drake

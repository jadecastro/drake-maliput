#pragma once

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts RoadCarState to a full 6-DOF EulerFloatingJointState.
template <typename T>
class RoadCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  RoadCarToEulerFloatingJoint(const maliput::geometry_api::RoadGeometry& road,
                              double lateral_offset) {
    this->set_name("RoadCarToEulerFloatingJoint");
    this->DeclareInputPort(systems::kVectorValued,
                           SimpleCarStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
    this->DeclareOutputPort(systems::kVectorValued,
                            EulerFloatingJointStateIndices::kNumCoordinates,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

    typedef systems::VectorBase<T> Base;
    const Base* const input_vector = this->EvalVectorInput(context, 0);
    DRAKE_ASSERT(input_vector != nullptr);
    const SimpleCarState<T>* const input_data =
        dynamic_cast<const SimpleCarState<T>*>(input_vector);
    DRAKE_ASSERT(input_data != nullptr);

    Base* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);
    EulerFloatingJointState<T>* const output_data =
        dynamic_cast<EulerFloatingJointState<T>*>(output_vector);
    DRAKE_ASSERT(output_data != nullptr);

    // Convert inputs into a joint position via road geometry.
    // Input X distance gets plotted along the 's' direction.
    // TODO(rico) this will never work without some road-geom state variables.

    // Proposed state vector:
    // junction index, segment index, lane index, s, r, h
    // From ji, si, li, we can get the current lane.
    // From s, r, h we build the current lane position.
    // From incoming velocity, we build IsolaneVelocity.
    // From the result of EMD, we get lane pose derivatives: sdot, rdot, hdot
    // TODO(rico): transit from one lane/segment to the the next?
    // From the lane pos, we get orientation back...

    output_data->set_x(x);
    output_data->set_y(y);
    output_data->set_z(z);
    output_data->set_roll(r);
    output_data->set_pitch(p);
    output_data->set_yaw(y);
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }
};

}  // namespace automotive
}  // namespace drake

#pragma once

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts EndlessRoadCarState to a full 6-DOF EulerFloatingJointState,
/// using the RoadGeometry on which the car is (presumably) driving.
template <typename T>
class EndlessRoadCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  // TODO(maddog)  Is a nice way for this to get the RoadGeometry from
  //               the connected car?
  EndlessRoadCarToEulerFloatingJoint(
      const maliput::utility::InfiniteCircuitRoad* road)
      : road_(road) {
    this->set_name("EndlessRoadCarToEulerFloatingJoint");
    this->DeclareInputPort(systems::kVectorValued,
                           EndlessRoadCarStateIndices::kNumCoordinates,
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
    const EndlessRoadCarState<T>* const input_data =
        dynamic_cast<const EndlessRoadCarState<T>*>(input_vector);
    DRAKE_ASSERT(input_data != nullptr);

    Base* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);
    EulerFloatingJointState<T>* const output_data =
        dynamic_cast<EulerFloatingJointState<T>*>(output_vector);
    DRAKE_ASSERT(output_data != nullptr);

    maliput::geometry_api::LanePosition lp(
        input_data->s(), input_data->r(), 0.);
    maliput::geometry_api::GeoPosition geo = road_->lane()->ToGeoPosition(lp);
    maliput::geometry_api::Rotation rot = road_->lane()->GetOrientation(lp);
    // TODO(maddog)  Deal with (rho_dot != 0) and with (sigma_dot < 0).

    output_data->set_x(geo.x_);
    output_data->set_y(geo.y_);
    output_data->set_z(geo.z_);
    output_data->set_roll(-rot.roll_); // TODO(maddog)  Why negative?????
    output_data->set_pitch(rot.pitch_);
    output_data->set_yaw(rot.yaw_);
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }

 private:
  const maliput::utility::InfiniteCircuitRoad* road_;
};

}  // namespace automotive
}  // namespace drake

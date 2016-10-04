#pragma once

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/automotive/maliput/geometry_api/lane.h"
#include "drake/automotive/maliput/geometry_api/road_geometry.h"
#include "drake/automotive/maliput/monolane/ignore.h"

namespace drake {
namespace automotive {

/// Converts RoadCarState to a full 6-DOF EulerFloatingJointState.
///
/// Until such time as discrete/hybrid state is supported, track 6-DOF state,
/// converting to road geometry and back each time we are asked for derivatives.
template <typename T>
class RoadCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  RoadCarToEulerFloatingJoint(const maliput::geometry_api::RoadGeometry& road,
                              double lateral_offset)
      : road_(road), lateral_offset_(lateral_offset) {
    this->set_name("RoadCarToEulerFloatingJoint");
    this->DeclareInputPort(systems::kVectorValued,
                           SimpleCarStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
    this->DeclareOutputPort(systems::kVectorValued,
                            EulerFloatingJointStateIndices::kNumCoordinates,
                            systems::kContinuousSampling);
  }

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

    // Obtain the state.
    const systems::VectorBase<T>& context_state =
        context.get_continuous_state()->get_state();
    const EulerFloatingJointState<T>* const state =
        dynamic_cast<const EulerFloatingJointState<T>*>(&context_state);
    DRAKE_ASSERT(state);

    // Obtain the input.
    const systems::VectorBase<T>* const vector_input =
        this->EvalVectorInput(context, 0);
    DRAKE_ASSERT(vector_input);
    const SimpleCarState<T>* const input =
        dynamic_cast<const SimpleCarState<T>*>(vector_input);
    DRAKE_ASSERT(input);
    // Convert inputs into a joint position via road geometry.
    // Input X distance gets plotted along the 's' direction.
    // TODO(rico) this will never work without some road-geom state variables.

    // Proposed state vector (PROVIDED we have hybrid/discrete support):
    // junction index, segment index, lane index, s, r, h
    // From ji, si, li, we can get the current lane.
    // From s, r, h we build the current lane position.
    // From incoming velocity, we build IsolaneVelocity.
    // From the result of EMD, we get lane pose derivatives: sdot, rdot, hdot
    // TODO(rico): transit from one lane/segment to the the next?
    // From the lane pos, we get orientation back...

    // The only continuous state rep we can support is the 6-dof joint
    // position, round-tripping through road space each time to evaluate.

    using namespace maliput::geometry_api;

    RoadPosition road_pos = road_.ToRoadPosition(
        {state->x(), state->y(), state->z()},
        {nullptr, {0, lateral_offset_, 0}});
    IsoLaneVelocity iso_velocity{input->velocity(), 0, 0};
    LanePosition pos_dot{};
    road_pos.lane_->EvalMotionDerivatives(
        road_pos.pos_, iso_velocity, &pos_dot);
    Rotation rot = road_pos.lane_->GetOrientation(road_pos.pos_);
    GeoPosition geo = road_pos.lane_->ToGeoPosition(road_pos.pos_);

    // Obtain the result structure.
    DRAKE_ASSERT(derivatives != nullptr);
    systems::VectorBase<T>* const vector_derivatives =
        derivatives->get_mutable_state();
    DRAKE_ASSERT(vector_derivatives);
    EulerFloatingJointState<T>* const rates =
        dynamic_cast<EulerFloatingJointState<T>*>(vector_derivatives);
    DRAKE_ASSERT(rates);

    // TODO(rico): given lane-relative pos_dot, rot, and geo, construct the
    // 6-dof derivative in geo. Presumably the canonical spelling for this in
    // Drake is some Eigen fest.
    ignore(rot);
    ignore(geo);
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

    // Obtain the state.
    const systems::VectorBase<T>& context_state =
        context.get_continuous_state()->get_state();
    const EulerFloatingJointState<T>* const state =
        dynamic_cast<const EulerFloatingJointState<T>*>(&context_state);
    DRAKE_ASSERT(state);

    // Obtain the output pointer.
    EulerFloatingJointState<T>* const output_data =
        dynamic_cast<EulerFloatingJointState<T>*>(
            output->GetMutableVectorData(0));
    DRAKE_ASSERT(output_data != nullptr);

    output_data->set_value(state->get_value());
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }

 private:
    const maliput::geometry_api::RoadGeometry& road_;
    const double lateral_offset_;
};

}  // namespace automotive
}  // namespace drake

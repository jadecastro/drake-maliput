#include "drake/automotive/automotive_simulator.h"

#include <limits>

#include <lcm/lcm-cpp.hpp>

#include "drake/automotive/endless_road_car.h"
#include "drake/automotive/endless_road_car_to_euler_floating_joint.h"
#include "drake/automotive/endless_road_oracle.h"
#include "drake/automotive/endless_road_traffic_car.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/endless_road_car_state_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_value_source.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include <iostream>

namespace drake {
namespace automotive {

using drake::multibody::joints::kRollPitchYaw;

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator()
    : AutomotiveSimulator(std::make_unique<lcm::DrakeLcm>()) {}

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator(
    std::unique_ptr<lcm::DrakeLcmInterface> lcm)
    : lcm_(std::move(lcm)) {}

template <typename T>
AutomotiveSimulator<T>::~AutomotiveSimulator() {
  // Forces the LCM instance to be destroyed before any of the subscribers are
  // destroyed.
  lcm_.reset();
}

template <typename T>
lcm::DrakeLcmInterface* AutomotiveSimulator<T>::get_lcm() {
  return lcm_.get();
}

template <typename T>
systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DRAKE_DEMAND(!started_);
  return builder_.get();
}

template <typename T>
const RigidBodyTree<T>& AutomotiveSimulator<T>::get_rigid_body_tree() {
  return *rigid_body_tree_;
}

template <typename T>
int AutomotiveSimulator<T>::AddSimpleCarFromSdf(
    const std::string& sdf_filename) {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, lcm_.get());
  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*command_subscriber, *simple_car);
  builder_->Connect(*simple_car, *coord_transform);
  AddPublisher(*simple_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform);
}

template <typename T>
int AutomotiveSimulator<T>::AddTrajectoryCarFromSdf(
    const std::string& sdf_filename, const Curve2<double>& curve, double speed,
    double start_time) {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  auto trajectory_car =
      builder_->template AddSystem<TrajectoryCar<T>>(curve, speed, start_time);
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*trajectory_car, *coord_transform);
  AddPublisher(*trajectory_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform);
}


template <typename T>
int AutomotiveSimulator<T>::AddEndlessRoadEgoCar(
    const std::string& id,
    const std::string& sdf_filename,
    double longitudinal_start,
    double lateral_offset,
    double speed,
    typename EndlessRoadCar<T>::ControlType control_type) {
  DRAKE_DEMAND(!started_);
  DRAKE_DEMAND((bool)endless_road_);
  const int vehicle_number = allocate_vehicle_number();

  auto endless_road_ego_car =
    builder_->template AddSystem<EndlessRoadCar<T>>(
      id, endless_road_.get(), control_type);
  auto coord_transform =
      builder_->template AddSystem<EndlessRoadCarToEulerFloatingJoint<T>>(
          endless_road_.get());

  // Save the desired initial state in order to initialize the Simulator later,
  // when we have a Simulator to initialize.
  EndlessRoadCarState<T>& initial_state =
    endless_road_ego_cars_[endless_road_ego_car];
  initial_state.set_s(longitudinal_start);
  initial_state.set_r(lateral_offset);
  initial_state.set_speed(speed);
  initial_state.set_heading(0.);

  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
    builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
                   "DRIVING_COMMAND", driving_command_translator, lcm_.get());
  builder_->Connect(*command_subscriber, *endless_road_ego_car);
  std::cerr << "Connected CommandSubscriber to EndlessRoadCar.\n";

  builder_->Connect(*endless_road_ego_car, *coord_transform);
  std::cerr << "Connected EndlessRoadCar to CoordTransform.\n";
  AddPublisher(*endless_road_ego_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform);
}


template <typename T>
int AutomotiveSimulator<T>::AddEndlessRoadTrafficCar(
    const std::string& id,
    const std::string& sdf_filename,
    const double s_init,
    const double r_init,
    const double v_init,
    const int num_cars) {
  DRAKE_DEMAND(!started_);
  DRAKE_DEMAND((bool)endless_road_);
  const int vehicle_number = allocate_vehicle_number();

  // TODO(jadecastro): Initialize heading also?
  // TODO(jadecastro): Do something smarter to set v_ref.
  auto endless_road_traffic_car =
    builder_->template AddSystem<EndlessRoadTrafficCar<T>>(
      id, num_cars, endless_road_.get(),
      s_init, r_init, v_init, 0., 30.);
  std::cerr << "   Simulator v_init: " << v_init << " \n";
  std::cerr << "   s_init: " << s_init << " \n";
  std::cerr << "Adding the CoordTransform...\n";
  auto coord_transform =
      builder_->template AddSystem<EndlessRoadCarToEulerFloatingJoint<T>>(
          endless_road_.get());

  // Save the desired initial state in order to initialize the
  // Simulator later, when we have a Simulator to initialize.
  // TODO(jadecastro): We don't actually need to save the initial
  // states anymore.
  EndlessRoadCarState<T>& initial_state =
    endless_road_traffic_cars_[endless_road_traffic_car];
  initial_state.set_s(s_init);
  initial_state.set_r(r_init);
  initial_state.set_speed(v_init);
  initial_state.set_heading(0.);

  std::cerr << "Attempting to connect to CoordTransform...\n";
  builder_->Connect(endless_road_traffic_car->get_output_port(0),
                    coord_transform->get_input_port(0));
  std::cerr << "Connected EndlessRoadTrafficCar to CoordTransform.\n";
  AddPublisher(*endless_road_traffic_car, vehicle_number);
  std::cerr << "Added EndlessRoadTrafficCar publisher.\n";
  AddPublisher(*coord_transform, vehicle_number);
  std::cerr << "Added CoordTransform publisher.\n";
  return AddSdfModel(sdf_filename, coord_transform);
}

template <typename T>
const maliput::utility::InfiniteCircuitRoad*
AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const maliput::api::RoadGeometry>* road,
    const maliput::api::LaneEnd& start,
    const std::vector<const maliput::api::Lane*>& path) {
  DRAKE_DEMAND(!started_);
  road_ = std::move(*road);
  endless_road_ = std::make_unique<maliput::utility::InfiniteCircuitRoad>(
      maliput::api::RoadGeometryId({"ForeverRoad"}),
      road_.get(), start, path);
  return endless_road_.get();
}


template <typename T>
int AutomotiveSimulator<T>::AddSdfModel(
    const std::string& sdf_filename,
    const SimpleCarToEulerFloatingJoint<T>* coord_transform) {
  const parsers::ModelInstanceIdTable table =
      parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
          sdf_filename, kRollPitchYaw, rigid_body_tree_.get());

  // TODO(liang.fok): Add support for SDF files containing more than one model.
  DRAKE_DEMAND(table.size() == 1);

  const int model_instance_id = table.begin()->second;
  rigid_body_tree_publisher_inputs_.push_back(
      std::make_pair(model_instance_id, coord_transform));
  return model_instance_id;
}

template <typename T>
int AutomotiveSimulator<T>::AddSdfModel(
    const std::string& sdf_filename,
    const EndlessRoadCarToEulerFloatingJoint<T>* coord_transform) {
  const parsers::ModelInstanceIdTable table =
      parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
          sdf_filename, kRollPitchYaw, rigid_body_tree_.get());

  // TODO(liang.fok): Add support for SDF files containing more than one model.
  DRAKE_DEMAND(table.size() == 1);

  const int model_instance_id = table.begin()->second;
  rigid_body_tree_publisher_inputs_.push_back(
      std::make_pair(model_instance_id, coord_transform));
  return model_instance_id;
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const TrajectoryCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const EndlessRoadCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EndlessRoadCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_ENDLESS_ROAD_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
              const EndlessRoadTrafficCar<T>& system, int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EndlessRoadCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_ENDLESS_ROAD_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system.get_output_port(0), publisher->get_input_port(0));
  std::cerr << "Connected EndlessRoadTrafficCar to Publisher.\n";
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const SimpleCarToEulerFloatingJoint<T>& system, int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE", translator,
          lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const EndlessRoadCarToEulerFloatingJoint<T>& system,
    int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddSystem(
    std::unique_ptr<systems::System<T>> system) {
  DRAKE_DEMAND(!started_);
  builder_->AddSystem(std::move(system));
}

template <typename T>
systems::System<T>& AutomotiveSimulator<T>::GetBuilderSystemByName(
    std::string name) {
  DRAKE_DEMAND(!started_);
  systems::System<T>* result{nullptr};
  for (systems::System<T>* system : builder_->GetMutableSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
const systems::System<T>& AutomotiveSimulator<T>::GetDiagramSystemByName(
    std::string name) const {
  DRAKE_DEMAND(started_);
  // Ask the diagram.
  const systems::System<T>* result{nullptr};
  for (const systems::System<T>* system : diagram_->GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
std::vector<int> AutomotiveSimulator<T>::GetModelJointStateSizes() const {
  std::vector<int> position_states;
  std::vector<int> velocity_states;

  for (const auto& model_info : rigid_body_tree_publisher_inputs_) {
    const int model_instance_id = model_info.first;
    int num_joint_positions{0};
    int num_joint_velocities{0};
    const std::vector<const RigidBody<T>*> bodies =
        rigid_body_tree_->FindModelInstanceBodies(model_instance_id);
    for (const auto& body : bodies) {
      num_joint_positions += body->getJoint().get_num_positions();
      num_joint_velocities += body->getJoint().get_num_velocities();
    }
    position_states.push_back(num_joint_positions);
    velocity_states.push_back(num_joint_velocities);
  }
  std::vector<int> result;
  result.reserve(position_states.size() + velocity_states.size());
  result.insert(result.end(), position_states.begin(), position_states.end());
  result.insert(result.end(), velocity_states.begin(), velocity_states.end());
  return result;
}

template <typename T>
void AutomotiveSimulator<T>::ConnectJointStateSourcesToVisualizer() {
  DRAKE_DEMAND(!started_);
  if (!rigid_body_tree_publisher_inputs_.empty()) {
    // Arithmetic for RigidBodyTreeLcmPublisher input sizing.  We have systems
    // that output an Euler floating joint state.  We want to mux them together
    // to feed RigidBodyTreeLcmPublisher.  We stack them up in joint order as
    // the position input to the publisher, and then also need to feed zeros
    // for all of the joint velocities.
    std::vector<int> joint_state_sizes = GetModelJointStateSizes();
    auto multiplexer = builder_->template AddSystem<systems::Multiplexer<T>>(
        joint_state_sizes);

    auto rigid_body_tree_publisher =
        builder_->template AddSystem<systems::DrakeVisualizer>(
            *rigid_body_tree_, lcm_.get());
    builder_->Connect(*multiplexer, *rigid_body_tree_publisher);
    std::cerr << "Connected Multiplexer to RBTreePublisher.\n";

    // Connects systems that provide joint positions to the mux position inputs.
    // Connects a zero-velocity source to each of the mux velocity inputs.
    const int num_models = rigid_body_tree_publisher_inputs_.size();
    for (int model_index = 0; model_index < num_models; ++model_index) {
      int model_instance_id{};
      const systems::System<T>* model_pose_system{};
      std::tie(model_instance_id, model_pose_system) =
          rigid_body_tree_publisher_inputs_[model_index];

      // Verifies that the current model instance is connected to the world via
      // an RPY floating joint.
      //
      // TODO(liang.fok) Support models that are connected to the world via
      // non RPY floating joints. See #3919.
      const int kRpyJointNumPos{6};
      const int kRpyJointNumVel{6};

      std::vector<int> base_body_indices =
          rigid_body_tree_->FindBaseBodies(model_instance_id);
      DRAKE_DEMAND(base_body_indices.size() == 1);
      const DrakeJoint& base_joint =
          rigid_body_tree_->bodies.at(base_body_indices.at(0))->getJoint();
      DRAKE_DEMAND(base_joint.is_floating());
      DRAKE_DEMAND(base_joint.get_num_positions() == kRpyJointNumPos);
      DRAKE_DEMAND(base_joint.get_num_velocities() == kRpyJointNumVel);

      // Determines the number of DOFs in the model instance.
      int num_position_dofs = joint_state_sizes.at(model_index);
      int num_velocity_dofs = joint_state_sizes.at(num_models + model_index);

      if (num_position_dofs == kRpyJointNumPos) {
        // The robot has no position DOFs beyond the floating joint DOFs. Thus,
        // model_pose_system can be connected directly to the multiplexer.
        builder_->Connect(model_pose_system->get_output_port(0),
                          multiplexer->get_input_port(model_index));
      } else {
        // The robot has additional DOFs beyond the floating joint DOFs. Thus,
        // we need to connect a system that provides additional joint position
        // state to the multiplexer. For now, we'll connect a zero-source
        // system.
        //
        // TODO(liang.fok): Enable non-floating joint DOFs to have non-zero
        // state see: #3919.
        const int num_reg_pos_dofs = num_position_dofs - kRpyJointNumPos;
        auto zero_position_source =
            builder_->template AddSystem<systems::ConstantVectorSource>(
                VectorX<T>::Zero(num_reg_pos_dofs).eval());
        auto position_mux =
            builder_->template AddSystem<systems::Multiplexer<T>>(
                std::vector<int>{kRpyJointNumPos, num_reg_pos_dofs});
        builder_->Connect(model_pose_system->get_output_port(0),
                          position_mux->get_input_port(0));
        builder_->Connect(zero_position_source->get_output_port(),
                          position_mux->get_input_port(1));
        builder_->Connect(position_mux->get_output_port(0),
                          multiplexer->get_input_port(model_index));
      }

      // Connects a zero-vector source for the velocity state.
      auto zero_velocity_source =
          builder_->template AddSystem<systems::ConstantVectorSource>(
              VectorX<T>::Zero(num_velocity_dofs).eval());

      builder_->Connect(zero_velocity_source->get_output_port(),
                        multiplexer->get_input_port(num_models + model_index));
    }
  }
}

template <typename T>
void AutomotiveSimulator<T>::Start() {
  DRAKE_DEMAND(!started_);
  // TODO(maddog)  This seems like hackery.
  // After all the moving parts (boxcars) have been added finally add
  // the static road (if any) to the RBT (otherwise the naive multiplexing
  // gets mucked up?).
  if (road_) {
    const double kGridUnit = 1.;  // meter
    maliput::utility::generate_urdf("/tmp", road_->id().id,
                                    road_.get(), kGridUnit);
    std::string urdf_filepath = std::string("/tmp/") + road_->id().id + ".urdf";
    parsers::urdf::AddModelInstanceFromUrdfFile(urdf_filepath,
                                                rigid_body_tree_.get());
    // NB: The road doesn't move, so we don't need to connect anything
    // to its joint.
  }

  // By this time, all model instances should have been added to the tree. Thus,
  // it should be safe to compile the tree.
  rigid_body_tree_->compile();

  std::cerr << "Attempting to connect states to visualizer...\n";
  ConnectJointStateSourcesToVisualizer();

  // Now that we have all the cars, construct an appropriately tentacled
  // EndlessRoadOracle and hook everything up.
  const int num_ego_cars = endless_road_ego_cars_.size();
  //const int num_traffic_cars = endless_road_traffic_cars_.size();

  // TODO(jadecastro): Deprecate the oracle for the ego cars.
  const EndlessRoadOracle<T>* oracle = nullptr;
  if (num_ego_cars) {
    oracle = builder_->template AddSystem<EndlessRoadOracle<T>>(
                                             endless_road_.get(), num_ego_cars);
  }

  std::cerr << "Attempting to connect each 'ego' car to each traffic car...\n";
  int i = 0;
  for (const auto& ego_car_item: endless_road_ego_cars_) {
    EndlessRoadCar<T>* ego_car = ego_car_item.first;

    // Every car is visible to the Oracle...
    builder_->Connect(ego_car->get_output_port(0), oracle->get_input_port(i));
    std::cerr << "Connected 'ego' car to oracle.\n";
    for (const auto& traffic_car_item: endless_road_traffic_cars_) {
      EndlessRoadTrafficCar<T>* traffic_car = traffic_car_item.first;
      builder_->Connect(ego_car->get_output_port(0),
                        traffic_car->get_input_port(i));
    }
    std::cerr << "Connected 'ego' car " << i << " to " <<
      endless_road_traffic_cars_.size() << " traffic cars.\n";
    // ...however, only IDM-controlled cars care about Oracle output.
    // TODO(maddog)  Optimization:  Oracle should not bother to compute
    //               output for cars which will ignore it.
    switch (ego_car->control_type()) {
    case EndlessRoadCar<T>::kNone: {
      break;  // No input.
    }
    case EndlessRoadCar<T>::kUser: {
      break;  // Already connected to controls.
    }
    default: { DRAKE_ABORT(); }
    }
    ++i;
  }

  int j = num_ego_cars;  // For debugging only.
  // TODO(jadecastro): Check the style guide on "globally-scoped" iterators.
  for (const auto& item: endless_road_traffic_cars_) {
    EndlessRoadTrafficCar<T>* traffic_car = item.first;

    int k = 0;
    std::cerr << "Number of car inputs: " <<
      traffic_car->get_num_input_ports() << ".\n";
    for (const auto& sensed_item: endless_road_traffic_cars_) {
      EndlessRoadTrafficCar<T>* sensed_traffic_car = sensed_item.first;
      // Traffic cars can sense all cars but themselves.
      if (sensed_traffic_car->get_id().compare(traffic_car->get_id())
          != 0) {
        std::cerr << "Attempting to connect traffic car " <<
            traffic_car->get_id() << " to a sensed traffic car " <<
            sensed_traffic_car->get_id() << ".\n";
        std::cerr << "  Input port: " << k << ".\n";
        builder_->Connect(sensed_traffic_car->get_output_port(0),
                          traffic_car->get_input_port(k));
        ++k;
      }
    }
    ++j;
  }

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);

  // Initialize the state of the EndlessRoadCars.
  // TODO(jadecastro): State-instantiation for "ego" cars is a bit
  // convoluted. Deprecate it.
  std::cerr << "Initializing the ego car states...\n";
  // TODO(jadecastro): EndlessRoadCars should be gotten directly from
  // the diagram.
  for (auto& pair : endless_road_ego_cars_) {
    systems::Context<T>* context =
        diagram_->GetMutableSubsystemContext(simulator_->get_mutable_context(),
                                             pair.first);
    systems::VectorBase<T>* context_state =
        context->get_mutable_continuous_state()->get_mutable_vector();
    EndlessRoadCarState<T>* const state =
        dynamic_cast<EndlessRoadCarState<T>*>(context_state);
    DRAKE_ASSERT(state);
    // TODO(maddog)  Is there a better way to copy all the fields?
    //               (I.e., until lcm_vector_gen.py makes an operator=()....)
    state->set_value(pair.second.get_value());
  }

  // TODO(jadecastro): We don't actually need to store a std::map of
  // cars and their initial conditions anymore.
  std::cerr << "Initializing the traffic car states...\n";
  //auto traffic_cars = diagram_->GetSystems();
  for (auto& item : endless_road_traffic_cars_) {
    //const EndlessRoadTrafficCar<T>* this_car =
    //  dynamic_cast<EndlessRoadTrafficCar<T>*>(car);
    EndlessRoadTrafficCar<T>* this_car = item.first;
    systems::Context<T>* context =
      diagram_->GetMutableSubsystemContext(simulator_->get_mutable_context(),
                                           this_car);
    systems::VectorBase<T>* state =
        context->get_mutable_continuous_state()->get_mutable_vector();
    // TODO(jadecastro): Instantiate states this way.
    //this_car->SetDefaultState(context);
    state->SetAtIndex(0, item.second.GetAtIndex(0));
    state->SetAtIndex(1, item.second.GetAtIndex(1));
    state->SetAtIndex(2, item.second.GetAtIndex(2));
    state->SetAtIndex(3, item.second.GetAtIndex(3));
  }
  //DRAKE_DEMAND(false);

  lcm_->StartReceiveThread();

  // TODO(jadecastro): Should simulator_->Intialize() allow for state
  // initialization?
  std::cerr << "Simulator initialize...\n";
  simulator_->Initialize();

  started_ = true;
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  std::cerr << "  ****** StepBy()..." << std::endl;
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  std::cerr << "  ****** StepBy() 2..." << std::endl;
  simulator_->StepTo(time + time_step);
  std::cerr << "  ****** StepBy() 3..." << std::endl;

  const int state_length = simulator_->
      get_context().get_continuous_state_vector().size();
  std::cerr << "  ****** StepBy() 4..." << std::endl;
  std::cerr << "      state vector length: " << state_length << std::endl;
  for (int i = 0; i < state_length; ++i) {
    std::cerr << "  StepBy : state at index 0 (should be s for cars): " <<
        simulator_->get_context().get_continuous_state_vector().GetAtIndex(i) <<
        std::endl;
  }

  int i = 0;
  for (auto sys : diagram_->GetSystems()) {
    //auto css =
    //    diagram_->GetSubsystemContext(simulator_->get_context(), sys);
    if (diagram_->GetSubsystemContext(simulator_->get_context(), sys).
        get_continuous_state_vector().size() > 0) {
      std::cerr << "  StepBy : subsystem " << i <<
          " sub state at index 0 (should be s for cars): " <<
          diagram_->GetSubsystemContext(simulator_->get_context(), sys).
          get_continuous_state_vector().GetAtIndex(0) << std::endl;
    }
    ++i;
  }
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DRAKE_DEMAND(!started_);
  return next_vehicle_number_++;
}

template class AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake

add_subdirectory(gen)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeAutomotive SOURCE_FILES
  create_trajectory_params.cc
  curve2.cc
  endless_road_car.cc
  endless_road_oracle.cc
  gen/driving_command.cc
  gen/endless_road_car_state.cc
  gen/endless_road_oracle_output.cc
  gen/euler_floating_joint_state.cc
  gen/idm_with_trajectory_agent_parameters.cc
  gen/idm_with_trajectory_agent_state.cc
  gen/simple_car_config.cc
  gen/simple_car_state.cc
  idm_with_trajectory_agent.cc
  maliput/monolane/arc_lane.cc
  maliput/monolane/branch_point.cc
  maliput/monolane/builder.cc
  maliput/monolane/junction.cc
  maliput/monolane/lane.cc
  maliput/monolane/line_lane.cc
  maliput/monolane/loader.cc
  maliput/monolane/road_geometry.cc
  maliput/monolane/segment.cc
  maliput/utility/generate_obj.cc
  maliput/utility/generate_urdf.cc
  maliput/utility/infinite_circuit_road.cc
  simple_car.cc
  trajectory_car.cc
  )
target_link_libraries(drakeAutomotive
  drakeCommon
  drakeSystemAnalysis
  drakeSystemFramework
  yaml-cpp
  )
pods_install_libraries(drakeAutomotive)
drake_install_headers(
  create_trajectory_params.h
  curve2.h
  endless_road_car.h
  endless_road_car-inl.h
  endless_road_oracle.h
  endless_road_oracle-inl.h
  idm_with_trajectory_agent.h
  simple_car.h
  simple_car_to_euler_floating_joint.h
  trajectory_car.h
  # N.B. The gen/*.h headers are installed by gen/CMakeLists.txt.
  )
pods_install_pkg_config_file(drake-automotive
  LIBS -ldrakeAutomotive
  REQUIRES
  VERSION 0.0.1)

add_subdirectory(maliput)

if(lcm_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)

  add_library_with_exports(LIB_NAME drakeAutomotiveLcm SOURCE_FILES
    automotive_simulator.cc
    gen/driving_command_translator.cc
    gen/endless_road_car_state_translator.cc
    gen/endless_road_oracle_output_translator.cc
    gen/euler_floating_joint_state_translator.cc
    gen/idm_with_trajectory_agent_state_translator.cc
    gen/simple_car_config_translator.cc
    gen/simple_car_state_translator.cc
    )
  target_link_libraries(drakeAutomotiveLcm
    drakeAutomotive
    drakeRigidBodyPlant
    )
  pods_install_libraries(drakeAutomotiveLcm)
  drake_install_headers(
    automotive_simulator.h
    # N.B. The gen/*.h headers are installed by gen/CMakeLists.txt.
    )
  pods_install_pkg_config_file(drake-automotive-lcm
    LIBS -ldrakeAutomotiveLcm
    REQUIRES
    VERSION 0.0.1)

  add_executable(automotive_demo automotive_demo.cc)
  target_link_libraries(automotive_demo drakeAutomotiveLcm drakeLCMTypes)

  add_executable(car_sim_lcm car_sim_lcm.cc)
  target_link_libraries(car_sim_lcm
      drakeAutomotiveLcm
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2)

  add_test(NAME car_sim_lcm COMMAND car_sim_lcm -simulation_sec 0.1)
endif()

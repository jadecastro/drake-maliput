add_executable(generate_obj_test generate_obj_test.cc)
target_link_libraries(
  generate_obj_test ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
drake_add_test(NAME generate_obj_test COMMAND generate_obj_test)

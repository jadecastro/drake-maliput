add_executable(hodgepodge hodgepodge.cc)
target_link_libraries(hodgepodge ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
drake_add_test(NAME hodgepodge COMMAND hodgepodge)

add_executable(monolane_lanes_test monolane_lanes_test.cc)
target_link_libraries(monolane_lanes_test ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
drake_add_test(NAME monolane_lanes_test COMMAND monolane_lanes_test)

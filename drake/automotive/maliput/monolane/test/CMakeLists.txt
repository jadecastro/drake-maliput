add_executable(hodgepodge hodgepodge.cc)
target_link_libraries(hodgepodge ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
drake_add_test(NAME hodgepodge COMMAND hodgepodge)

add_library(flowlessly_misc statistics.cc utils.cc)

target_link_libraries(flowlessly_misc flowlessly_graphs)

# Build tests
add_executable(utils_tests utils_tests.cc)
target_link_libraries(utils_tests glog gflags ${gtest_LIBRARY} ${gtest_MAIN_LIBRARY} pthread flowlessly_misc flowlessly_graphs)
add_test(utils_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/utils_tests)

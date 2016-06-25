add_library(flowlessly_graphs graph.cc adjacency_map_graph.cc graph_generator.cc)

target_link_libraries(flowlessly_graphs glog gflags flowlessly_misc)

# Build tests
add_executable(adjacency_map_graph_tests adjacency_map_graph_tests.cc)
target_link_libraries(adjacency_map_graph_tests gflags glog ${gtest_LIBRARY} ${gtest_MAIN_LIBRARY} pthread flowlessly_misc flowlessly_graphs)
add_test(adjacency_map_graph_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/adjacency_map_graph_tests)

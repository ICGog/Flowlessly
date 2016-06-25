# Build flow schedule
add_executable(flow_scheduler flow_scheduler.cc)

target_link_libraries(flow_scheduler LINK_PUBLIC gflags glog boost_chrono boost_system boost_thread boost_timer flowlessly_misc flowlessly_graphs flowlessly_solvers)

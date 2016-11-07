# Build flow schedule

include(graphs/CMakeLists.txt)
include(misc/CMakeLists.txt)
include(solvers/CMakeLists.txt)

#####################################################################
# Object collections
add_library(graphs OBJECT ${GRAPHS_SRCS})
add_dependencies(graphs gtest gflags glog)
add_library(misc OBJECT ${MISC_SRCS})
add_dependencies(misc gtest gflags glog pthread)
add_library(solvers OBJECT ${SOLVERS_SRCS})
add_dependencies(solvers gtest gflags glog pthread boost_thread boost_system)

#####################################################################
# Flow scheduler

set(FLOW_SCHEDULER_SRC
  flow_scheduler.cc
  )

add_executable(flow_scheduler ${FLOW_SCHEDULER_SRC}
  $<TARGET_OBJECTS:misc>
  $<TARGET_OBJECTS:graphs>
  $<TARGET_OBJECTS:solvers>
)

add_dependencies(flow_scheduler gflags glog boost_chrono boost_system
  boost_thread boost_timer)

target_link_libraries(flow_scheduler LINK_PUBLIC gflags glog boost_chrono
  boost_system boost_thread boost_timer)
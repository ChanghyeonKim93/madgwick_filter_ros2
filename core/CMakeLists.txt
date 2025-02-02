message("   -- [DIRECTORY]: " core)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})

set(SRC ${SRC}
  madgwick_filter.cc
)

add_executable(
  madgwick_filter_node 
  node.cc ${SRC}
)
target_link_libraries(
  madgwick_filter_node 
  ${Eigen3_LIBRARIES} ${OpenCV_LIBS}
)
ament_target_dependencies(
  madgwick_filter_node 
  rclcpp sensor_msgs geometry_msgs nav_msgs message_filters 
)
install(
  TARGETS madgwick_filter_node
  DESTINATION lib/${PROJECT_NAME}) # install executable
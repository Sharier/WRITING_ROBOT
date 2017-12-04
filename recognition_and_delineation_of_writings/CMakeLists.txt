cmake_minimum_required(VERSION 2.8.3)

project(recognition_and_delineation_of_writings)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  image_transport
  roscpp
  rospy
  std_srvs
  std_msgs
  sensor_msgs
  cv_bridge
  rostime
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)

add_executable(starting_node src/starting_node.cpp)
add_dependencies(starting_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(starting_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
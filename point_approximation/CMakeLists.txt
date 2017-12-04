cmake_minimum_required(VERSION 2.8.3)

project(point_approximation)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  coordinate_exposure
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
  ${coordinate_exposure_INCLUDE_DIRS}
)

add_executable(point_chaining src/point_chaining.cpp)
add_dependencies(point_chaining ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_chaining
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${coordinate_exposure_LIBRARIES}
)
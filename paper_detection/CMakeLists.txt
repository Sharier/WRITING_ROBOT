cmake_minimum_required(VERSION 2.8.3)

project(paper_detection)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  message_filters
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  point.msg
  point_set.msg
  point_sets.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_conversion
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(point_conversion src/point_conversion.cpp)
add_dependencies(point_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_conversion
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
)

add_executable(rectangle_detection src/rectangle_detection.cpp)
add_dependencies(rectangle_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rectangle_detection
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_conversion
)

add_executable(centering src/centering.cpp)
add_dependencies(centering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(centering
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_conversion
)

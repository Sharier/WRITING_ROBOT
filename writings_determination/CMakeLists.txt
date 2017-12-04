cmake_minimum_required(VERSION 2.8.3)

project(writings_determination)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  coordinate_exposure
)

add_service_files(
  FILES
  image_exchange.srv
  image_check.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)

add_executable(skeleton src/skeleton.cpp)
add_dependencies(skeleton ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(skeleton
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(binary src/binary.cpp)
add_dependencies(binary ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(binary
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(rim src/rim.cpp)
add_dependencies(rim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rim
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(writings_determination src/writings_determination.cpp)
add_dependencies(writings_determination ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(writings_determination
  ${catkin_LIBRARIES}
)

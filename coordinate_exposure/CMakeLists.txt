cmake_minimum_required(VERSION 2.8.3)

project(coordinate_exposure)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  roscpp
  rospy
  message_generation
  std_msgs
)

add_message_files(
  FILES
  coordinate.msg
  coordinate_set.msg
  coordinate_sets.msg
)

add_service_files(
  FILES
  coordinate_sets_exchange.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coordinate_conversion
  CATKIN_DEPENDS message_runtime roscpp rospy
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(coordinate_conversion src/coordinate_conversion.cpp)
add_dependencies(coordinate_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coordinate_conversion
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
)

add_executable(coordinate_saving src/coordinate_saving.cpp)
add_dependencies(coordinate_saving ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coordinate_saving
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${OpenCV_LIBRARIES}
  coordinate_conversion
)

add_executable(routing src/routing.cpp)
add_dependencies(routing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(routing
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${OpenCV_LIBRARIES}
  coordinate_conversion
)

add_executable(coordinate_calculation src/coordinate_calculation.cpp)
add_dependencies(coordinate_calculation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coordinate_calculation
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${rospy_LIBRARIES}
  ${OpenCV_LIBRARIES}
  coordinate_conversion
)

cmake_minimum_required(VERSION 3.0.2)
project(ranger_round_trip)

add_compile_options(
  -std=c++14
  -Wall
  -Wextra
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib
    move_base_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  round_trip_mission_node
  src/round_trip_mission_node.cpp
)

add_dependencies(
  round_trip_mission_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  round_trip_mission_node
  ${catkin_LIBRARIES}
)


add_executable(
  cmd_vel_mux_node
  src/cmd_vel_mux_node.cpp
)

add_dependencies(
  cmd_vel_mux_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  cmd_vel_mux_node
  ${catkin_LIBRARIES}
)

#install(
#  TARGETS round_trip_mission_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
install(
  TARGETS
    round_trip_mission_node
    cmd_vel_mux_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

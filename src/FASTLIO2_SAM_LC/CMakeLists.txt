cmake_minimum_required(VERSION 3.0.2)
project(fastlio)


ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Loop-closure map_builder_node needs GTSAM. Keep localization buildable when
# the dev container does not provide GTSAM.
find_package(GTSAM QUIET)



add_service_files(
  FILES
  SlamReLoc.srv
  SaveMap.srv
  MapConvert.srv
  SlamHold.srv
  SlamStart.srv
  SlamRelocCheck.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fastlio
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

set(COMMON_SRC src/commons.cpp)
set(KDTREE_SRC include/ikd-Tree/ikd_Tree.cpp)
set(LIOBUILDER_SRC_LIST src/lio_builder/imu_processor.cpp src/lio_builder/lio_builder.cpp)
set(ICPLOCALIZER_SRC src/localizer/icp_localizer.cpp)
set(SRC_LIST ${COMMON_SRC} ${KDTREE_SRC} ${LIOBUILDER_SRC_LIST} ${ICPLOCALIZER_SRC})



if(GTSAM_FOUND)
  add_executable(map_builder_node src/map_builder_node.cpp ${SRC_LIST})

  target_link_libraries(map_builder_node
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam
  )
else()
  message(WARNING "GTSAM not found, skip building map_builder_node with loop closure")
endif()

add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
add_dependencies(localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(localizer_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(temp_node src/temp_node.cpp ${SRC_LIST})
add_dependencies(temp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(temp_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(init_pose_node src/init_pose_node.cpp)
add_dependencies(init_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(init_pose_node
  ${catkin_LIBRARIES}
)

add_executable(save_map_node src/save_map_node.cpp)
add_dependencies(save_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(save_map_node
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(underwater_inspection)

add_compile_options(-std=c++11 -g) # -g for gdb

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  # greensea_msgs
  inspection_planner_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  tf
)

find_package(octomap REQUIRED)
# find_package(Eigen3 REQUIRED)

add_message_files(
   FILES
   ViewpointInfo.msg
   MultiViewpointInfo.msg
)

add_service_files(
   FILES
   DepleteBattery.srv
   BatteryUsage.srv
   CarveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  inspection_planner_msgs
  underwater_inspection
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES underwater_inspection
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  include/underwater_inspection
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES})
# ${EIGEN3_LIBRARY_DIRS}

add_executable(${PROJECT_NAME}_node src/map_carving.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  # ${EIGEN3_LIBRARIES}

)


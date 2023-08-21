cmake_minimum_required(VERSION 2.8.3)
project(pcl_sensor_data)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
	std_msgs
  message_generation
  laser_geometry
  tf
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Generate services in the 'srv' folder
add_service_files(
   FILES
   avgZDistance.srv
   angleAdjust.srv
   laserMax.srv
   ProcessPointcloud.srv
   StartRecording.srv
   StopRecording.srv
   RestartRecording.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
 	DEPENDENCIES
 	std_msgs
 )
catkin_package(
   #INCLUDE_DIRS include
#  LIBRARIES pcl_sensor_data
CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs message_runtime tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

INSTALL(FILES
    DESTINATION include)

add_executable(access_data src/access_data.cpp)
add_dependencies(access_data ${pcl_sensor_data_EXPORTED_TARGETS} $($(PROJECT_NAME)_EXPORTED_TARGETS) $(catkin_EXPORTED_TARGETS))
target_link_libraries(access_data ${catkin_LIBRARIES})

add_executable(test_node src/test_node.cpp)
add_dependencies(test_node ${pcl_sensor_data_EXPORTED_TARGETS} $($(PROJECT_NAME)_EXPORTED_TARGETS) $(catkin_EXPORTED_TARGETS))
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(laser_processor src/laser_processor.cpp)
add_dependencies(laser_processor ${pcl_sensor_data_EXPORTED_TARGETS} $($(PROJECT_NAME)_EXPORTED_TARGETS) $(catkin_EXPORTED_TARGETS))
target_link_libraries(laser_processor ${catkin_LIBRARIES})

add_executable(laser_playground src/laser_playground.cpp)
add_dependencies(laser_playground ${pcl_sensor_data_EXPORTED_TARGETS} $($(PROJECT_NAME)_EXPORTED_TARGETS) $(catkin_EXPORTED_TARGETS))
target_link_libraries(laser_playground ${catkin_LIBRARIES})

# add_executable(seg_viewer src/seg_viewer.cpp)
# add_dependencies(seg_viewer ${pcl_sensor_data_EXPORTED_TARGETS} $($(PROJECT_NAME)_EXPORTED_TARGETS) $(catkin_EXPORTED_TARGETS))
# target_link_libraries(seg_viewer ${catkin_LIBRARIES})

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
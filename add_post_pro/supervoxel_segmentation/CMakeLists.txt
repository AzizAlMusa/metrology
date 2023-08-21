cmake_minimum_required(VERSION 3.0.2)
project(supervoxel_segmentation)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(pcl_basic src/pcl_basic.cpp)
add_executable(voxel_example src/voxel_example.cpp)
add_executable(vcc_node src/vcc_node.cpp src/segment.cpp)
add_executable(registration_demo src/registration_demo.cpp)
add_executable(reg_demo2 src/reg_demo2.cpp)
add_executable(reg_demo3 src/reg_demo3.cpp)
add_executable(reg_demo4 src/reg_demo4.cpp)

target_include_directories(vcc_node PRIVATE include)



add_dependencies(pcl_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_basic  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_dependencies(voxel_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_example  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(vcc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vcc_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(registration_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(registration_demo  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_dependencies(reg_demo2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reg_demo2  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(reg_demo3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reg_demo3  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(reg_demo4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reg_demo4  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_dependencies(segment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(segment  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


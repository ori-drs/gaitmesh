cmake_minimum_required(VERSION 3.10)
project(gaitmesh_ros)

# Use c++11
set(CMAKE_CXX_STANDARD 11)

set(CATKIN_DEPENDENCIES
  roscpp
  roslib
  visualization_msgs
  pcl_ros
  recast_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES} message_generation)

# Point Cloud Library
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  CloudMesh.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  pcl_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare the gaitmesh_ros library which contains the point cloud annotation features
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/lvr2_reconstruction.cpp     # Las Vegas Reconstruction
  src/meshlab_reconstruction.cpp  # Meshlab Reconstruction
  src/pcl/PclMeshSampling.cpp     # Modified copy of PCL's mesh sampling methods
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## annotate_cloud_gaits: single point-cloud annotation tool
add_executable(annotate_cloud_gaits src/annotate_cloud_gaits.cpp)
add_dependencies(annotate_cloud_gaits ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(annotate_cloud_gaits ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## annotate_cloud_gaits_continuous (temporary - duplicates code!)
add_executable(annotate_cloud_gaits_continuous src/annotate_cloud_gaits_continuous.cpp)
add_dependencies(annotate_cloud_gaits_continuous ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(annotate_cloud_gaits_continuous ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## publish_cloud_meshes
add_executable(publish_cloud_meshes src/publish_cloud_meshes.cpp)
add_dependencies(publish_cloud_meshes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_cloud_meshes ${PROJECT_NAME} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} annotate_cloud_gaits annotate_cloud_gaits_continuous publish_cloud_meshes
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install config, launch, rviz
install(DIRECTORY config/ data/ launch/ rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wabian_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

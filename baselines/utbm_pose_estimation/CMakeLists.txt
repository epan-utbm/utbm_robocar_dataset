cmake_minimum_required(VERSION 2.8.3)
project(utbm_pose_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hector_pose_estimation_core nodelet sensor_msgs geometry_msgs nav_msgs tf message_filters)

# Find hector_timing and use it, if available
find_package(hector_timing QUIET)
if(hector_timing_FOUND)
  list(APPEND catkin_INCLUDE_DIRS ${hector_timing_INCLUDE_DIRS})
  list(APPEND catkin_LIBRARIES ${hector_timing_LIBRARIES})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES utbm_pose_estimation_nodelet utbm_pose_estimation_node
    CATKIN_DEPENDS hector_pose_estimation_core nodelet sensor_msgs geometry_msgs nav_msgs tf message_filters
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(utbm_pose_estimation_node
  src/pose_estimation_node.cpp
)
target_link_libraries(utbm_pose_estimation_node ${catkin_LIBRARIES})

add_library(utbm_pose_estimation_nodelet
  src/pose_estimation_nodelet.cpp
)
target_link_libraries(utbm_pose_estimation_nodelet utbm_pose_estimation_node ${catkin_LIBRARIES})

add_executable(pose_estimation
  src/main.cpp
)
target_link_libraries(pose_estimation utbm_pose_estimation_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS utbm_pose_estimation_nodelet utbm_pose_estimation_node pose_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY
  launch
  rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

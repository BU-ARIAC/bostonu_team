cmake_minimum_required(VERSION 2.8.3)
project(hw_example)

find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  sensor_msgs
  std_srvs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
)

# Adding PCL lib
find_package(PCL REQUIRED)
# Setting include, lib directories and definitions 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS} )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package()

###########
## Build ##
###########
# /usr/include/pcl-1.8/pcl/point_cloud.h
include_directories(include/bostonu_team ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(hw_example_node src/hw_example_node.cpp src/part_mgmt.cpp src/order_mgmt.cpp src/robot_mgmt.cpp src/robot_kitting.cpp src/robot_gantry.cpp)
add_dependencies(hw_example_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hw_example_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  script/hw_example_node.py
#  script/tf2_example.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS hw_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/bostonu_team/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
 config/bu_user_config_all.yaml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ariac_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

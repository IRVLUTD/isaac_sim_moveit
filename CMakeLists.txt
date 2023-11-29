cmake_minimum_required(VERSION 3.0.2)
project(isaac_sim_moveit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  controller_manager
  transmission_interface
  hardware_interface
  sensor_msgs
  joint_limits_interface
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
    arm_interface
   CATKIN_DEPENDS 
    std_msgs
    roscpp
    controller_manager
    transmission_interface
    hardware_interface
    sensor_msgs
    joint_limits_interface
    trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(arm_interface src/arm_hardware_interface.cpp)
target_link_libraries(arm_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(arm_interface ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finished

add_executable(fetch_hardware_interface src/fetch_hardware_interface.cpp)
target_link_libraries(fetch_hardware_interface 
  arm_interface 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
add_dependencies(fetch_hardware_interface ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish

catkin_install_python(PROGRAMS
  scripts/panda_combined_joints_publisher.py
  scripts/fetch_combined_joints_publisher.py
  scripts/prepare_simulated_robot_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
cmake_minimum_required(VERSION 2.8.3)
project(events)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  std_msgs
  robotis_controller_msgs
  op3_walking_module_msgs
  op3_action_module_msgs

  message_generation
  vision
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_python_setup()

add_message_files(
  FILES
  Track.msg
  remote_control_action.msg
  remote_control_head.msg
  remote_control_walking.msg
  new_pos.msg
  ScanLimits.msg
)

add_service_files(
  FILES
  get_map.srv
)

generate_messages()

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS
    rospy
    roslib
    std_msgs
    robotis_controller_msgs
    op3_walking_module_msgs
    op3_action_module_msgs
    cmake_modules
    robotis_math
    message_runtime
  DEPENDS
    Boost
    EIGEN3
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

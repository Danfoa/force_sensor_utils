cmake_minimum_required(VERSION 2.8.3)
project(force_sensor_utils)

add_compile_options(-std=c++11)

find_package(catkin 
  REQUIRED 
  
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    genmsg
    tf2
    tf2_ros
    tf2_geometry_msgs
    actionlib 
    actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    AxisRange.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY 
    action 
  FILES 
    ForceTorqueMonitoring.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
    actionlib_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  DEPENDS system_lib 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    actionlib_msgs 
    message_runtime
    message_filters
    sensor_msgs
    tf2
    tf2_ros
  DEPENDS
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Tutorial executables.

add_executable(wrench_transformer src/wrench_transformer.cpp)
target_link_libraries(wrench_transformer ${catkin_LIBRARIES})
add_dependencies(wrench_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(PROGRAMS
  scripts/invite_move_group_python_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

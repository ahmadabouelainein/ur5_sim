cmake_minimum_required(VERSION 3.0.2)
project(ur5_ros_gazebo)

##############################################################################
#  Dependencies
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  actionlib
  control_msgs   
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs sensor_msgs
                 geometry_msgs actionlib control_msgs
)

include_directories(
  include               # <- add if you create headers later
  ${catkin_INCLUDE_DIRS}
)

add_executable(sine_joint_action_client src/sine_joint_action_client.cpp)
target_link_libraries(sine_joint_action_client ${catkin_LIBRARIES})

install(TARGETS
        sine_joint_action_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

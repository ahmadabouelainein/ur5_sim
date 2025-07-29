cmake_minimum_required(VERSION 3.0.2)
project(ur_motion_api)             
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs
  ur5_ros_gazebo
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs
                 actionlib actionlib_msgs control_msgs ur5_ros_gazebo
)

catkin_install_python(
  PROGRAMS scripts/motion_api.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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


if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

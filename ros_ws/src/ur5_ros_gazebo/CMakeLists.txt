cmake_minimum_required(VERSION 3.0.2)
project(ur5_ros_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs          # sine_joint_publisher uses these headers
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_sgs sensor_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sine_joint_publisher src/sine_joint_publisher.cpp)
target_link_libraries(sine_joint_publisher ${catkin_LIBRARIES})

install(TARGETS sine_joint_publisher
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(ur5_ros_gazebo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#  Dependencies (catkin packages + non‑catkin libs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  actionlib
  control_msgs
  kdl_parser            # for building the KDL::Chain from URDF
  ur_kinematics         # exports libur5_kin for the analytic solver
)

find_package(orocos_kdl REQUIRED)       # non‑catkin, needed for IK/FK math


#  catkin_package — export catkin deps, list non‑catkin in DEPENDS

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs sensor_msgs
                 geometry_msgs actionlib control_msgs kdl_parser ur_kinematics
  DEPENDS orocos_kdl
)


#  Include directories

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)


#  Libraries and executables

add_library(motion_lib
  src/kdl_ik.cpp
  src/traj_utils.cpp
  src/motion_library.cpp
)

# Link motion_lib against Orocos‑KDL, catkin libs, and the analytic UR solver
target_link_libraries(motion_lib
  ${orocos_kdl_LIBRARIES}
  ur5_kin
  ${catkin_LIBRARIES}
)

add_executable(motion_demo_node src/motion_node.cpp)
target_link_libraries(motion_demo_node
  motion_lib
  ${catkin_LIBRARIES}
)

add_executable(sine_joint_action_client src/sine_joint_action_client.cpp)
target_link_libraries(sine_joint_action_client
  ${catkin_LIBRARIES}
)


#  Installation rules

install(TARGETS
        motion_lib
        motion_demo_node
        sine_joint_action_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

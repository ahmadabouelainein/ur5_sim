cmake_minimum_required(VERSION 3.0.2)
project(ur5_ros_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  control_msgs
  message_generation
  kdl_parser
  ur_kinematics
)
find_package(orocos_kdl REQUIRED)

add_action_files(
  DIRECTORY action
  FILES      MoveJoint.action
             MoveLinear.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
                 trajectory_msgs control_msgs actionlib actionlib_msgs
                 message_runtime kdl_parser ur_kinematics
  DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(motion_lib
  src/kdl_ik.cpp
  # src/traj_utils.cpp
  src/motion_library.cpp
)
target_link_libraries(motion_lib
  ${orocos_kdl_LIBRARIES}
  ur5_kin
  ${catkin_LIBRARIES}
)
add_dependencies(motion_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motion_action_server src/motion_action_server.cpp)
target_link_libraries(motion_action_server motion_lib ${catkin_LIBRARIES})
add_dependencies(motion_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motion_demo_node src/motion_node.cpp)
target_link_libraries(motion_demo_node motion_lib ${catkin_LIBRARIES})
add_dependencies(motion_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sine_joint_action_client src/sine_joint_action_client.cpp)
target_link_libraries(sine_joint_action_client ${catkin_LIBRARIES})
add_dependencies(sine_joint_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
        motion_lib
        motion_action_server
        motion_demo_node
        sine_joint_action_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <ur5_ros_gazebo/motion_library.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_demo_node");
  ros::NodeHandle nh;

  /* --- build KDL chain from URDF --------------------------------------- */
  urdf::Model urdf;  urdf.initParam("robot_description");
  KDL::Tree tree;    kdl_parser::treeFromUrdfModel(urdf, tree);
  KDL::Chain chain;  tree.getChain("base_link", "tool0", chain);

  /* --- joint names in controller order --------------------------------- */
  std::vector<std::string> names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

  /* --- create MotionLibrary (analytic IK) ------------------------------ */
  MotionLibrary lib(chain, names, 0.02);
  ROS_INFO_STREAM("MotionLibrary ready with DOF = " << names.size());

  ros::spin();
  return 0;
}

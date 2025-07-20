#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <ur5_ros_gazebo/motion_library.hpp>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"motion_demo_node");
  ros::NodeHandle nh;

  /* --- build KDL chain from URDF ----------------------------------------- */
  urdf::Model urdf;  urdf.initParam("robot_description");
  KDL::Tree tree;    kdl_parser::treeFromUrdfModel(urdf,tree);
  KDL::Chain chain;  tree.getChain("base_link","tool0",chain);

  /* --- extract joint limits & names -------------------------------------- */
  unsigned dof = chain.getNrOfJoints();
  KDL::JntArray qmin(dof), qmax(dof);
  std::vector<std::string> names; names.reserve(dof);

  unsigned idx=0;
  for(const auto& seg: chain.segments){
    const auto& j = seg.getJoint();
    if(j.getType()==KDL::Joint::None) continue;
    auto uj = urdf.getJoint(j.getName());
    qmin(idx) = uj->limits->lower;
    qmax(idx) = uj->limits->upper;
    names.push_back(j.getName());
    ++idx;
  }

  /* --- create planner ---------------------------------------------------- */
  MotionLibrary lib(chain, qmin, qmax, names, 0.01);

  ROS_INFO("MotionLibrary ready with %u DOF.", dof);
  ros::spin();
  return 0;
}

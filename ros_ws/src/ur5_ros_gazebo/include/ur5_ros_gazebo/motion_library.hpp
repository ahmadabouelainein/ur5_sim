#pragma once
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <ur5_ros_gazebo/kdl_ik.hpp>

#include <vector>
#include <string>

class MotionLibrary
{
public:
  /* ── full constructor: chain + joint limits ───────────────────────────── */
  MotionLibrary(const KDL::Chain&               chain,
                const KDL::JntArray&            qmin,
                const KDL::JntArray&            qmax,
                const std::vector<std::string>& joint_names,
                double                          dt = 0.01);

  /* ── convenience overload: assumes ±π limits everywhere ───────────────── */
  MotionLibrary(const KDL::Chain&               chain,
                const std::vector<std::string>& joint_names,
                double                          dt = 0.01);

  trajectory_msgs::JointTrajectory
  generateJointMove(const std::vector<double>& q_start,
                    const std::vector<double>& q_goal,
                    double v_max, double a_max) const;

  trajectory_msgs::JointTrajectory
  generateLinearMove(const geometry_msgs::Pose& pose_start,
                     const geometry_msgs::Pose& pose_goal,
                     double v_lin, double a_lin,
                     const std::vector<double>& q_seed) const;

private:
  ik::Solver               ik_;
  std::vector<std::string> joint_names_;
  double                   dt_;
};

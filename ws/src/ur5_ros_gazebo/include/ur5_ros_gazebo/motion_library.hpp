#pragma once
#include <ur_kinematics/ur_kin.h>
#include <kdl/chain.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>

class MotionLibrary
{
public:
  MotionLibrary(const KDL::Chain&               chain,
                const std::vector<std::string>& joint_names,
                double                          dt = 0.02);

  trajectory_msgs::JointTrajectory generateJointMove(
        const std::vector<double>& q_curr,
        const std::vector<double>& q_start,
        const std::vector<double>& q_target,
        double v_max, double a_max,
        double tolerance_rad = 1e-3) const;

  trajectory_msgs::JointTrajectory generateLinearMove(
        const geometry_msgs::Pose& P0,
        const geometry_msgs::Pose& P1,
        double      v_lin,
        double      a_lin,
        const std::vector<double>& q_seed) const;


private:
  std::vector<std::string> joint_names_;
  double                   dt_;
};

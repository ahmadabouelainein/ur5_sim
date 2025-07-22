#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>

using FJTAction   = control_msgs::FollowJointTrajectoryAction;
using FJTGoal     = control_msgs::FollowJointTrajectoryGoal;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_sine_action_client");
  ros::NodeHandle nh;

  // --- connect to the controller's action server -------------
  actionlib::SimpleActionClient<FJTAction> ac(
      "/eff_joint_traj_controller/follow_joint_trajectory", true);

  ROS_INFO("Waiting for /eff_joint_traj_controller action server...");
  if (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Action server not available – "
              "did eff_joint_traj_controller load?");
    return 1;
  }
  ROS_INFO("Connected.");

  const std::vector<std::string> joints = {
      "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
      "wrist_1_joint","wrist_2_joint","wrist_3_joint"};

  const double amp = 0.5;     // rad
  const double freq = 0.1;    // Hz
  const double loop_hz = 100; // same as YAML publish_rate
  ros::Rate rate(loop_hz);

  const double t0 = ros::Time::now().toSec();

  while (ros::ok())
  {
    double t = ros::Time::now().toSec() - t0;

    FJTGoal goal;
    goal.trajectory.joint_names = joints;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(joints.size());
    p.velocities.resize(joints.size());
    p.time_from_start = ros::Duration(1.0/loop_hz);  // 10 ms window

    for (size_t i = 0; i < joints.size(); ++i) {
      double phase = i * M_PI / 6.0;
      p.positions[i]  = amp * std::sin(2*M_PI*freq*t + phase);
      p.velocities[i] = amp * 2*M_PI*freq * std::cos(2*M_PI*freq*t + phase);
    }
    goal.trajectory.points.push_back(p);

    ac.sendGoal(goal);                // non‑blocking
    rate.sleep();
  }
  return 0;
}

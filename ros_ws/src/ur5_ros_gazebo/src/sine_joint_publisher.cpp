#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ur5_sine_joint_publisher");
  ros::NodeHandle nh;

  const std::vector<std::string> joint_names = {
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint"};

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/pos_joint_traj_controller/command", 1);

  const double amplitude = 0.5;      // rad
  const double frequency = 0.1;      // Hz
  const double publish_rate = 100.0; // Hz
  ros::Rate rate(publish_rate);

  const double start_time = ros::Time::now().toSec();
  while (ros::ok()) {
    double t = ros::Time::now().toSec() - start_time;

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    traj.header.stamp = ros::Time::now();

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(joint_names.size());
    p.velocities.resize(joint_names.size());
    p.time_from_start = ros::Duration(1.0 / publish_rate);

    for (size_t i = 0; i < joint_names.size(); ++i) {
      double phase = i * M_PI / 6.0;  // small offset between joints
      p.positions[i]   = amplitude * sin(2 * M_PI * frequency * t + phase);
      p.velocities[i]  = amplitude * 2 * M_PI * frequency * cos(2 * M_PI * frequency * t + phase);
    }
    traj.points.push_back(p);
    traj_pub.publish(traj);
    rate.sleep();
  }
  return 0;
}
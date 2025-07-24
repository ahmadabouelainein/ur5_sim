// ur5_ros_gazebo/test/test_motion_action_server.cpp
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ur5_ros_gazebo/MoveJointAction.h>
#include <sensor_msgs/JointState.h>

// Typedefs for convenience
using FJTServer = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;
using FJTClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
using MoveJointClient = actionlib::SimpleActionClient<ur5_ros_gazebo::MoveJointAction>;

static const char* JOINT_STATE_TOPIC = "/joint_states";
static const char* FOLLOW_TRAJ_NS    = "/eff_joint_traj_controller/follow_joint_trajectory";
static const char* MOVE_JOINT_NS     = "move_joint";

class MotionActionServerTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  std::shared_ptr<FJTServer> fake_traj_server_;

  virtual void SetUp() override
  {
    // 1) start fake trajectory controller that immediately returns SUCCESS
    fake_traj_server_.reset(new FJTServer(
      nh_, FOLLOW_TRAJ_NS,
      [&](const control_msgs::FollowJointTrajectoryGoalConstPtr&){  
        control_msgs::FollowJointTrajectoryResult res;  
        res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;  
        fake_traj_server_->setSucceeded(res);
      },
      false /* auto_start */));
    fake_traj_server_->start();

    // 2) give everything a moment
    ros::Duration(1.0).sleep();
  }

  void publishJointState()
  {
    // publish a default zero-state for 6 joints
    sensor_msgs::JointState js;
    js.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
               "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    js.position.assign(6, 0.0);
    ros::Publisher pub = nh_.advertise<sensor_msgs::JointState>(
      JOINT_STATE_TOPIC, 1, /* latch */ true);
    // wait for subscriber
    ros::Duration(0.5).sleep();
    pub.publish(js);
    ros::Duration(0.5).sleep();
  }
};

TEST_F(MotionActionServerTest, MoveJointSucceeds)
{
  // 3) launch the server under test
  // (will be launched via rostest .test wrapper)

  // 4) publish a joint state
  publishJointState();

  // 5) send a MoveJointAction goal
  MoveJointClient mjc(MOVE_JOINT_NS, true);
  ASSERT_TRUE(mjc.waitForServer(ros::Duration(5.0)));

  ur5_ros_gazebo::MoveJointGoal goal;
  // use simple target: move joint[0] from 0 to 0.5 rad
  goal.q_start.assign(6, 0.0);
  goal.q_target = goal.q_start;
  goal.q_target[0] = 0.5;
  goal.v_max = 0.1;
  goal.a_max = 0.2;

  mjc.sendGoal(goal);
  bool finished = mjc.waitForResult(ros::Duration(10.0));
  ASSERT_TRUE(finished) << "Action did not finish";
  auto result = mjc.getResult();
  EXPECT_TRUE(result->success);
}


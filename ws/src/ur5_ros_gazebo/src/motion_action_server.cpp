#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <ur5_ros_gazebo/motion_library.hpp>
#include <ur5_ros_gazebo/MoveJointAction.h>
#include <ur5_ros_gazebo/MoveLinearAction.h>

using FJTAction       = control_msgs::FollowJointTrajectoryAction;
using MoveJointASMsg  = ur5_ros_gazebo::MoveJointAction;
using MoveLinearASMsg = ur5_ros_gazebo::MoveLinearAction;

/**
 * @brief Provides ROS action servers for joint-space and Cartesian linear motions.
 */
class MotionActionServer
{
public:
  /**
   * @brief Construct and initialize the action servers and trajectory client.
   * @param nh ROS node handle
   */
  explicit MotionActionServer(ros::NodeHandle& nh)
    : as_joint_(nh, "move_joint",  false)
    , as_linear_(nh, "move_linear", false)
    , traj_ac_("/eff_joint_traj_controller/follow_joint_trajectory", true)
  {
    urdf::Model urdf;
    urdf.initParam("robot_description");
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf, tree);
    tree.getChain("base_link", "tool0", chain_);

    joint_names_ = {
      "shoulder_pan_joint", "shoulder_lift_joint",
      "elbow_joint",         "wrist_1_joint",
      "wrist_2_joint",       "wrist_3_joint"
    };
    
    lib_.reset(new MotionLibrary(chain_, joint_names_, 0.04));

    js_sub_ = nh.subscribe<sensor_msgs::JointState>(
      "/joint_states", 1,
      [this](const sensor_msgs::JointState::ConstPtr& msg) {
        latest_state_ = *msg;
        have_state_ = true;
      }
    );

    as_joint_.registerGoalCallback  ( [this]{ onJointGoal();  } );
    as_joint_.registerPreemptCallback( [this]{ traj_ac_.cancelAllGoals(); as_joint_.setPreempted(); } );
    as_linear_.registerGoalCallback ( [this]{ onLinearGoal(); } );
    as_linear_.registerPreemptCallback([this]{ traj_ac_.cancelAllGoals(); as_linear_.setPreempted(); } );

    as_joint_.start();
    as_linear_.start();

    ROS_INFO("Waiting for trajectory controller action server ...");
    traj_ac_.waitForServer();
    ROS_INFO("MotionActionServer ready.");
  }

private:
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<MotionLibrary> lib_;
  actionlib::SimpleActionClient<FJTAction> traj_ac_;
  actionlib::SimpleActionServer<MoveJointASMsg>  as_joint_;
  actionlib::SimpleActionServer<MoveLinearASMsg> as_linear_;
  sensor_msgs::JointState latest_state_;
  bool have_state_{false};
  ros::Subscriber js_sub_;

  /**
   * @brief Handle MoveJoint goals: generate a joint-space trajectory and execute it.
   */
  void onJointGoal()
{
  auto goal = as_joint_.acceptNewGoal();

  // 1) Make sure we have a joint state
  if (!have_state_) {
    auto js = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/joint_states", ros::Duration(5.0)
    );
    if (!js) {
      ROS_ERROR("onJointGoal: timed out waiting for /joint_states");
      as_joint_.setAborted();
      return;
    }
    latest_state_ = *js;
    have_state_   = true;
  }

  // 2) Reorder into controller order
  std::vector<double> q_curr(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = std::find(
      latest_state_.name.begin(),
      latest_state_.name.end(),
      joint_names_[i]
    );
    if (it == latest_state_.name.end()) {
      ROS_ERROR_STREAM("onJointGoal: joint '" << joint_names_[i]
                       << "' not found in latest_state_.name");
      as_joint_.setAborted();
      return;
    }
    q_curr[i] = latest_state_.position[
      std::distance(latest_state_.name.begin(), it)
    ];
  }

  // 3) Generate trajectory
  auto traj = lib_->generateJointMove(
    q_curr,
    goal->q_start,
    goal->q_target,
    goal->v_max,
    goal->a_max,
    0.1
  );
  if (traj.points.empty()) {
    ROS_ERROR("onJointGoal: generateJointMove returned empty trajectory");
    as_joint_.setAborted();
    return;
  }

  // 4) Send and wait
  control_msgs::FollowJointTrajectoryGoal ctl;
  for (const auto& name : joint_names_)
    {
      control_msgs::JointTolerance tol;
      tol.name         = name;
      tol.position     = 2;
      tol.velocity     = 5;
      tol.acceleration = 1;
      ctl.path_tolerance.push_back(tol);
    }
    for (const auto& name : joint_names_)
    {
      control_msgs::JointTolerance tol;
      tol.name         = name;
      tol.position     = 0.01;
      tol.velocity     = 0.02;
      tol.acceleration = 0.0;
      ctl.goal_tolerance.push_back(tol);
    }
  ctl.goal_time_tolerance = ros::Duration(100);
  ctl.trajectory = traj;
  traj_ac_.sendGoal(ctl, boost::bind(&MotionActionServer::doneCb, this, _1, _2),
            actionlib::SimpleActionClient<FJTAction>::SimpleActiveCallback(),
             boost::bind(&MotionActionServer::fbCb, this, _1));
  bool finished = traj_ac_.waitForResult();
  if (!finished) {
    ROS_ERROR("onJointGoal: trajectory execution timed out");
    traj_ac_.cancelAllGoals();
    as_joint_.setAborted();
    return;
  }

  // 5) Check result
  bool success = (traj_ac_.getState() ==
                  actionlib::SimpleClientGoalState::SUCCEEDED);
  if (!success) {
    ROS_ERROR_STREAM("onJointGoal: controller returned state: "
                     << traj_ac_.getState().toString());
    ur5_ros_gazebo::MoveJointResult r; r.success = false;
    as_joint_.setAborted(r);
  } else {
    ur5_ros_gazebo::MoveJointResult r; r.success = true;
    as_joint_.setSucceeded(r);
  }
}

void fbCb(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr& fb)
{
  for (size_t i = 0; i < fb->joint_names.size(); ++i)
  {
    ROS_INFO("  %s:  desired=%.3f  actual=%.3f  error=%.3f",
      fb->joint_names[i].c_str(),
      fb->desired.positions[i],
      fb->actual.positions[i],
      fb->error.positions[i]);
  }
}
void doneCb(
  const actionlib::SimpleClientGoalState& state,
  const control_msgs::FollowJointTrajectoryResultConstPtr& res)
{
  ROS_INFO("Final state: %s", state.toString().c_str());
  ROS_INFO(" error_code = %d", res->error_code);

  switch (res->error_code)
  {
    case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL:
      ROS_INFO(" → trajectory succeeded!");
      break;
    case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
      ROS_WARN(" → path tolerance violated!");
      break;
    case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
      ROS_WARN(" → goal tolerance violated!");
      break;
    default:
      ROS_ERROR(" → trajectory aborted for error code %d", res->error_code);
  }
}


void onLinearGoal()
{
  auto goal = as_linear_.acceptNewGoal();

  // 1) Ensure we have a valid joint state
  if (!have_state_) {
    auto js = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/joint_states", ros::Duration(5.0)
    );
    if (!js) {
      ROS_ERROR("onLinearGoal: timed out waiting for /joint_states");
      ur5_ros_gazebo::MoveLinearResult r; r.success = false;
      as_linear_.setAborted(r);
      return;
    }
    latest_state_ = *js;
    have_state_   = true;
  }

  // 2) Build current joint vector in controller order
  std::vector<double> q_curr(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = std::find(
      latest_state_.name.begin(),
      latest_state_.name.end(),
      joint_names_[i]
    );
    if (it == latest_state_.name.end()) {
      ROS_ERROR_STREAM("onLinearGoal: joint '" << joint_names_[i]
                       << "' not found in latest_state_");
      ur5_ros_gazebo::MoveLinearResult r; r.success = false;
      as_linear_.setAborted(r);
      return;
    }
    q_curr[i] = latest_state_.position[
      std::distance(latest_state_.name.begin(), it)
    ];
  }

  // 4) Generate Cartesian trajectory with the joint-vector seed
  auto traj = lib_->generateLinearMove(
    goal->pose_start,
    goal->pose_goal,
    goal->v_lin,
    goal->a_lin,
    q_curr
  );
  if (traj.points.empty()) {
    ROS_ERROR("onLinearGoal: generateLinearMove returned empty trajectory");
    ur5_ros_gazebo::MoveLinearResult r; r.success = false;
    as_linear_.setAborted(r);
    return;
  }

  // 5) Send trajectory and stream feedback
  control_msgs::FollowJointTrajectoryGoal ctl;
  ctl.trajectory = traj;
  traj_ac_.sendGoal(ctl);

  ros::Time start = ros::Time::now();
  ros::Duration total = traj.points.back().time_from_start;
  while (!traj_ac_.getState().isDone() && ros::ok()) {
    ros::Duration elapsed = ros::Time::now() - start;
    float pct = std::min(1.0f,
                         static_cast<float>(elapsed.toSec() / total.toSec()));
    ur5_ros_gazebo::MoveLinearFeedback fb;
    fb.percent_complete = pct * 100.0f;
    as_linear_.publishFeedback(fb);
    ros::Duration(0.1).sleep();
  }

  // 6) Report final result
  bool success = (traj_ac_.getState() ==
                  actionlib::SimpleClientGoalState::SUCCEEDED);
  ur5_ros_gazebo::MoveLinearResult res;
  res.success = success;
  if (success)
    as_linear_.setSucceeded(res);
  else {
    ROS_ERROR_STREAM("onLinearGoal: trajectory execution failed, state="
                     << traj_ac_.getState().toString());
    as_linear_.setAborted(res);
  }
}


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_action_server");
  ros::NodeHandle nh;
  MotionActionServer server(nh);
  ros::spin();
  return 0;
}

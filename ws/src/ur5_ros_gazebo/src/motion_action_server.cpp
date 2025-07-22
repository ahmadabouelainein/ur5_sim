// ──────────────────────────────────────────────────────────────────────────
//  motion_action_server.cpp   (ROS Noetic, analytic IKFast)
// ──────────────────────────────────────────────────────────────────────────
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>

#include <ur5_ros_gazebo/motion_library.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <ur5_ros_gazebo/MoveJointAction.h>
#include <ur5_ros_gazebo/MoveLinearAction.h>

using FJTAction       = control_msgs::FollowJointTrajectoryAction;
using MoveJointASMsg  = ur5_ros_gazebo::MoveJointAction;
using MoveLinearASMsg = ur5_ros_gazebo::MoveLinearAction;

/* ───────────────────────── MotionActionServer ─────────────────────────── */
class MotionActionServer
{
public:
  explicit MotionActionServer(ros::NodeHandle& nh)
  : as_joint_ (nh, "move_joint",  false),
    as_linear_(nh, "move_linear", false),
    traj_ac_ ("/eff_joint_traj_controller/follow_joint_trajectory", true)
  {
    /* -------- build KDL chain (link order only) ----------------------- */
    urdf::Model urdf;  urdf.initParam("robot_description");
    KDL::Tree tree;    kdl_parser::treeFromUrdfModel(urdf, tree);
    tree.getChain("base_link", "tool0", chain_);

    joint_names_ = { "shoulder_pan_joint", "shoulder_lift_joint",
                     "elbow_joint", "wrist_1_joint",
                     "wrist_2_joint", "wrist_3_joint" };

    lib_.reset(new MotionLibrary(chain_, joint_names_, 0.04));  // 25 Hz

    js_sub_ = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 1,
        [this](const sensor_msgs::JointState::ConstPtr& msg)
        { latest_state_ = *msg; have_state_ = true; });

    /* -------- action‑server callbacks --------------------------------- */
    as_joint_.registerGoalCallback ([this]{ onJointGoal();  });
    as_joint_.registerPreemptCallback([this]{ traj_ac_.cancelAllGoals();
                                             as_joint_.setPreempted(); });

    as_linear_.registerGoalCallback ([this]{ onLinearGoal(); });
    as_linear_.registerPreemptCallback([this]{ traj_ac_.cancelAllGoals();
                                              as_linear_.setPreempted(); });

    as_joint_.start();
    as_linear_.start();

    /* -------- wait for hardware controller ---------------------------- */
    ROS_INFO("Waiting for trajectory controller action server…");
    traj_ac_.waitForServer();
    ROS_INFO("MotionActionServer ready.");
  }

private:
  /* ---------------- joint goal ---------------------------------------- */
  void onJointGoal()
  {
    auto goal = as_joint_.acceptNewGoal();

    if (!have_state_) {
      auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
                  "/joint_states", ros::Duration(5.0));
      if (!msg) { as_joint_.setAborted(); return; }
      latest_state_ = *msg; have_state_ = true;
    }

    /* reorder current pose to controller order ------------------------ */
    std::vector<double> q_curr(joint_names_.size(), 0.0);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto it = std::find(latest_state_.name.begin(),
                          latest_state_.name.end(),
                          joint_names_[i]);
      if (it == latest_state_.name.end()) { as_joint_.setAborted(); return; }
      q_curr[i] = latest_state_.position[std::distance(latest_state_.name.begin(), it)];
    }

    /* goal start / target already in controller order */
    auto traj = lib_->generateJointMove(q_curr,
                                        goal->q_start,
                                        goal->q_target,
                                        goal->v_max,
                                        goal->a_max,
                                        0.1 /* tol */);
    if (traj.points.empty()) { as_joint_.setAborted(); return; }

    control_msgs::FollowJointTrajectoryGoal g; g.trajectory = traj;
    traj_ac_.sendGoal(g);

    traj_ac_.waitForResult();
    ur5_ros_gazebo::MoveJointResult res;
    res.success = (traj_ac_.getState() ==
                   actionlib::SimpleClientGoalState::SUCCEEDED);
    res.success ? as_joint_.setSucceeded(res)
                : as_joint_.setAborted(res);
  }

  /* ---------------- linear goal --------------------------------------- */
  void onLinearGoal()
  {
    auto goal = as_linear_.acceptNewGoal();

    auto traj = lib_->generateLinearMove(goal->pose_start,
                                         goal->pose_goal,
                                         goal->v_lin,
                                         goal->a_lin,
                                         goal->q_seed);
    if (traj.points.empty()) {
      ur5_ros_gazebo::MoveLinearResult r; r.success = false;
      as_linear_.setAborted(r); return;
    }
    control_msgs::FollowJointTrajectoryGoal g; g.trajectory = traj;
    traj_ac_.sendGoal(g);

    traj_ac_.waitForResult();
    ur5_ros_gazebo::MoveLinearResult res;
    res.success = (traj_ac_.getState() ==
                   actionlib::SimpleClientGoalState::SUCCEEDED);
    as_linear_.setSucceeded(res);
  }

  /* ---------------- members ------------------------------------------- */
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<MotionLibrary> lib_;

  actionlib::SimpleActionClient<FJTAction> traj_ac_;
  actionlib::SimpleActionServer<MoveJointASMsg>  as_joint_;
  actionlib::SimpleActionServer<MoveLinearASMsg> as_linear_;

  sensor_msgs::JointState latest_state_;
  bool have_state_{false};
  ros::Subscriber js_sub_;
};

/* ----------------------------- main ------------------------------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_action_server");
  ros::NodeHandle nh;
  MotionActionServer server(nh);
  ros::spin();
  return 0;
}

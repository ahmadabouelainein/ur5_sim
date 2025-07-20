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

using FJTAction      = control_msgs::FollowJointTrajectoryAction;
using MoveJointASMsg = ur5_ros_gazebo::MoveJointAction;
using MoveLinearASMsg= ur5_ros_gazebo::MoveLinearAction;

class MotionActionServer
{
public:
  explicit MotionActionServer(ros::NodeHandle& nh)
  : as_joint_ (nh, "move_joint",  false),
    as_linear_(nh, "move_linear", false),
    traj_ac_ ("/eff_joint_traj_controller/follow_joint_trajectory", true)
  {
    /* --- build MotionLibrary from URDF once ------------------------------ */
    urdf::Model urdf; urdf.initParam("robot_description");
    KDL::Tree tree;   kdl_parser::treeFromUrdfModel(urdf, tree);
    tree.getChain("base_link","tool0", chain_);

    const unsigned dof = chain_.getNrOfJoints();
    KDL::JntArray qmin(dof), qmax(dof);
    unsigned i=0;
    for(const auto& seg: chain_.segments){
      const auto& j = seg.getJoint();
      if(j.getType()==KDL::Joint::None) continue;
      auto uj = urdf.getJoint(j.getName());
      qmin(i) = uj->limits->lower;
      qmax(i) = uj->limits->upper;
      // joint_names_.push_back(j.getName());
      ++i;
    }
    joint_names_ = { "shoulder_pan_joint", "shoulder_lift_joint",
                 "elbow_joint", "wrist_1_joint",
                 "wrist_2_joint", "wrist_3_joint" };
    lib_.reset(new MotionLibrary(chain_, qmin, qmax, joint_names_, 0.01));
    js_sub_ = nh.subscribe<sensor_msgs::JointState>(
            "/joint_states", 1,
            [this](const sensor_msgs::JointState::ConstPtr& msg)
            { latest_state_ = *msg; have_state_ = true; });

    /* --- action‑server callbacks ---------------------------------------- */
    as_joint_.registerGoalCallback ([this]{ onJointGoal();  });
    as_joint_.registerPreemptCallback([this]{ traj_ac_.cancelAllGoals(); as_joint_.setPreempted(); });

    as_linear_.registerGoalCallback ([this]{ onLinearGoal(); });
    as_linear_.registerPreemptCallback([this]{ traj_ac_.cancelAllGoals(); as_linear_.setPreempted(); });

    as_joint_.start();
    as_linear_.start();

    /* --- wait for trajectory controller --------------------------------- */
    ROS_INFO("Waiting for trajectory controller action server...");
    traj_ac_.waitForServer();
    ROS_INFO("MotionActionServer ready.");
  }

private:
  /* ---------- joint goal handler ---------------------------------------- */
  void onJointGoal()
  {
    auto goal = as_joint_.acceptNewGoal();
    if (!have_state_)
    {
      ROS_WARN("motion_action_server: waiting for first /joint_states …");
      auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
                  "/joint_states",  ros::Duration(5.0));   // 5‑s timeout
      if (!msg) {                      // still nothing → abort goal gracefully
        ROS_ERROR("No joint state received, aborting joint action.");
        as_joint_.setAborted(); return;
      }
      latest_state_ = *msg;  have_state_ = true;
    }

  std::vector<double> q_curr = latest_state_.position;   // length == 6

  auto traj = lib_->generateJointMove(
               q_curr,                   // current robot pose
               goal->q_start,            // desired start
               goal->q_target,           // desired goal
               goal->v_max,
               goal->a_max,
              0.1);

    control_msgs::FollowJointTrajectoryGoal g; g.trajectory = traj;
    traj_ac_.sendGoal(g);

    ros::Rate r(10);
    while(!traj_ac_.getState().isDone())
    {
      if(as_joint_.isPreemptRequested()) return;
      ur5_ros_gazebo::MoveJointFeedback fb;
      fb.percent_complete = 50.0f;          // placeholder
      as_joint_.publishFeedback(fb);
      r.sleep();
    }
    ur5_ros_gazebo::MoveJointResult res;
    res.success = (traj_ac_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED);
    as_joint_.setSucceeded(res);
  }

  /* ---------- linear goal handler --------------------------------------- */
  void onLinearGoal()
  {
    auto goal = as_linear_.acceptNewGoal();
    auto traj = lib_->generateLinearMove(goal->pose_start,
                                         goal->pose_goal,
                                         goal->v_lin,
                                         goal->a_lin,
                                         goal->q_seed);
    if(traj.points.empty()){
      ur5_ros_gazebo::MoveLinearResult r; r.success=false;
      as_linear_.setAborted(r); return;
    }
    control_msgs::FollowJointTrajectoryGoal g; g.trajectory = traj;
    traj_ac_.sendGoal(g);

    traj_ac_.waitForResult();
    ur5_ros_gazebo::MoveLinearResult res;
    res.success = (traj_ac_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED);
    as_linear_.setSucceeded(res);
  }

  /* members */
  KDL::Chain chain_;                      /* kinematic chain               */
  std::vector<std::string> joint_names_;  /* joint name order              */
  std::unique_ptr<MotionLibrary> lib_;    /* trajectory & IK helper        */

  actionlib::SimpleActionClient<FJTAction> traj_ac_;
  actionlib::SimpleActionServer<MoveJointASMsg>  as_joint_;
  actionlib::SimpleActionServer<MoveLinearASMsg> as_linear_;
  sensor_msgs::JointState  latest_state_;
  bool have_state_{false};
  ros::Subscriber js_sub_;

};

/* ----------------------------- main -------------------------------------- */
int main(int argc,char** argv)
{
  ros::init(argc,argv,"motion_action_server");
  ros::NodeHandle nh;
  MotionActionServer server(nh);
  ros::spin();
  return 0;
}

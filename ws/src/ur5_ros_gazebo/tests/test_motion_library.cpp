#include <ur5_ros_gazebo/motion_library.hpp>
#include <gtest/gtest.h>

using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

/* ---------- helpers ---------------------------------------------------- */
static MotionLibrary makeDummyLib()
{
  /* 6‑DOF dummy chain (lengths don't matter for logic unit tests) */
  KDL::Chain c;
  for (int i = 0; i < 6; ++i)
    c.addSegment( KDL::Segment( KDL::Joint( KDL::Joint::RotZ ) ) );

  std::vector<std::string> names = {
      "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
      "wrist_1_joint","wrist_2_joint","wrist_3_joint" };

  return MotionLibrary(c, names, 0.02);
}

/* ---------- tests ------------------------------------------------------ */
TEST(MotionLibrary, JointMoveEndPointsMatch)
{
  auto lib = makeDummyLib();
  std::vector<double> q0{0,0,0,0,0,0};
  std::vector<double> q1{1,2,3,4,5,6};

  JointTrajectory traj = lib.generateJointMove(q0,q0,q1,0.5,1.0);
  ASSERT_FALSE(traj.points.empty());

  const auto& last = traj.points.back();
  for (size_t i=0;i<q1.size();++i)
    EXPECT_NEAR(last.positions[i], q1[i], 1e-6);
}

TEST(MotionLibrary, TimeMonotonicIncreasing)
{
  auto lib = makeDummyLib();
  std::vector<double> q0(6,0.0), q1(6,1.0);
  auto traj = lib.generateJointMove(q0,q0,q1,0.5,1.0);

  ros::Duration prev(0.0);
  for(const auto& pt: traj.points)
  {
    EXPECT_GT(pt.time_from_start.toSec(), prev.toSec());
    prev = pt.time_from_start;
  }
}

/* add more: cartesian line move, IK fail returns empty trajectory, etc. */

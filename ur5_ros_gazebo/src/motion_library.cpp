#include <ur5_ros_gazebo/motion_library.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>

#include <algorithm>
#include <cmath>

/* ───────────────────────── helper utilities ──────────────────────────── */
namespace {
double profileTime(double dq, double v, double a)
{
  dq = std::fabs(dq);
  const double t_acc  = v / a;
  const double dq_acc = 0.5 * a * t_acc * t_acc;
  return (dq <= 2.0 * dq_acc)
         ? 2.0 * std::sqrt(dq / a)
         : 2.0 * t_acc + (dq - 2.0 * dq_acc) / v;
}

KDL::Rotation slerpKDL(const KDL::Rotation& R0,
                       const KDL::Rotation& R1,
                       double s)
{
  double x0,y0,z0,w0; R0.GetQuaternion(x0,y0,z0,w0);
  double x1,y1,z1,w1; R1.GetQuaternion(x1,y1,z1,w1);
  tf2::Quaternion q0(x0,y0,z0,w0), q1(x1,y1,z1,w1);
  tf2::Quaternion qi = q0.slerp(q1, s);
  return KDL::Rotation::Quaternion(qi.x(), qi.y(), qi.z(), qi.w());
}
} // anonymous namespace

/* ───────────────────────── constructors ───────────────────────────────── */
MotionLibrary::MotionLibrary(const KDL::Chain&  chain,
                             const KDL::JntArray& qmin,
                             const KDL::JntArray& qmax,
                             const std::vector<std::string>& names,
                             double dt)
: ik_(chain, qmin, qmax),
  joint_names_(names),
  dt_(dt)
{}

MotionLibrary::MotionLibrary(const KDL::Chain&               chain,
                             const std::vector<std::string>& names,
                             double                          dt)
: MotionLibrary(                                                // ← delegates
      chain,
      /* qmin */ [&chain]{
                   KDL::JntArray arr(chain.getNrOfJoints());
                   for (unsigned i = 0; i < arr.rows(); ++i) arr(i) = -M_PI;
                   return arr;
                 }(),
      /* qmax */ [&chain]{
                   KDL::JntArray arr(chain.getNrOfJoints());
                   for (unsigned i = 0; i < arr.rows(); ++i) arr(i) =  M_PI;
                   return arr;
                 }(),
      names,
      dt)
{}

/* ───────────────────────── joint‑space move ───────────────────────────── */
trajectory_msgs::JointTrajectory
MotionLibrary::generateJointMove(const std::vector<double>& q_curr,
                                 const std::vector<double>& q_start,
                                 const std::vector<double>& q_target,
                                 double v_max, double a_max,
                                 double tolerance_rad /* = 1e-3 */) const
{
  const size_t dof = q_curr.size();
  auto build_segment = [&](const std::vector<double>& qa,
                           const std::vector<double>& qb)
  {
    std::vector<double> durations(dof);
    for (size_t i = 0; i < dof; ++i)
      durations[i] = profileTime(qb[i] - qa[i], v_max, a_max);

    double T = *std::max_element(durations.begin(), durations.end());

    trajectory_msgs::JointTrajectory seg;
    seg.joint_names = joint_names_;

    for (double t = 0.0; t <= T + 1e-9; t += dt_) {
      double s = std::clamp(t / T, 0.0, 1.0);
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions.resize(dof);
      for (size_t i = 0; i < dof; ++i)
        pt.positions[i] = qa[i] + s * (qb[i] - qa[i]);
      pt.time_from_start = ros::Duration(t);
      seg.points.push_back(pt);
    }
    return seg;
  };

  /* ── 1) optional pre‑segment ───────────────────────────────────────── */
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names_;
  bool withinTolerance = true;
  {
  for (size_t i = 0; i < q_curr.size(); ++i)
    if (std::fabs(q_curr[i] - q_start[i]) > tolerance_rad) withinTolerance=false;
  }
  if (!withinTolerance) {
    auto seg0 = build_segment(q_curr, q_start);
    traj.points.insert(traj.points.end(),
                       seg0.points.begin(), seg0.points.end());
  }

  /* ── 2) main segment (q_start → q_target) ──────────────────────────── */
  auto seg1 = build_segment(q_start, q_target);

  /* adjust time‑from‑start of seg1 if we already added seg0 */
  ros::Duration offset = traj.points.empty() ?
                         ros::Duration(0) :
                         traj.points.back().time_from_start;
  for (auto& pt : seg1.points)
    pt.time_from_start += offset;

  traj.points.insert(traj.points.end(),
                     seg1.points.begin(), seg1.points.end());

  return traj;
}


/* ───────────────────────── Cartesian line move ────────────────────────── */
trajectory_msgs::JointTrajectory
MotionLibrary::generateLinearMove(const geometry_msgs::Pose& P0,
                                  const geometry_msgs::Pose& P1,
                                  double v_lin, double a_lin,
                                  const std::vector<double>& q_seed) const
{
  /* Convert poses to KDL frames */
  tf2::Quaternion q0_tf, q1_tf;
  tf2::fromMsg(P0.orientation, q0_tf);
  tf2::fromMsg(P1.orientation, q1_tf);

  KDL::Frame F0(KDL::Rotation::Quaternion(q0_tf.x(), q0_tf.y(), q0_tf.z(), q0_tf.w()),
                KDL::Vector(P0.position.x, P0.position.y, P0.position.z));
  KDL::Frame F1(KDL::Rotation::Quaternion(q1_tf.x(), q1_tf.y(), q1_tf.z(), q1_tf.w()),
                KDL::Vector(P1.position.x, P1.position.y, P1.position.z));

  const double dist = (F1.p - F0.p).Norm();
  const double T    = profileTime(dist, v_lin, a_lin);

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names_;

  std::vector<double> q_prev = q_seed;

  for (double t = 0.0; t <= T + 1e-9; t += dt_) {
    double s = std::clamp(t / T, 0.0, 1.0);
    KDL::Vector    P = F0.p + s * (F1.p - F0.p);
    KDL::Rotation  R = slerpKDL(F0.M, F1.M, s);
    KDL::Frame     F(R, P);

    std::vector<double> q_sol;
    if (!ik_.solve(F, q_prev, q_sol)) {
      ROS_WARN_STREAM("IK failed at t=" << t << "; aborting.");
      return trajectory_msgs::JointTrajectory{};
    }
    q_prev = q_sol;

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = q_sol;
    pt.time_from_start = ros::Duration(t);
    traj.points.push_back(pt);
  }
  return traj;
}
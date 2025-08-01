#include <ur5_ros_gazebo/motion_library.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>

#include <algorithm>
#include <cmath>

/* helper: simple clamp without C++17 */
static inline double clamp01(double x)
{ return (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x; }

/* trapezoidal profile duration */
static double profileTime(double dq, double v, double a)
{
  dq = std::fabs(dq);
  const double t_acc  = v / a;
  const double dq_acc = 0.5 * a * t_acc * t_acc;
  return (dq <= 2.0 * dq_acc)
         ? 2.0 * std::sqrt(dq / a)
         : 2.0 * t_acc + (dq - 2.0 * dq_acc) / v;
}

/* slerp between two KDL rotations */
static KDL::Rotation slerpKDL(const KDL::Rotation& R0,
                              const KDL::Rotation& R1,
                              double s)
{
  double x0,y0,z0,w0; R0.GetQuaternion(x0,y0,z0,w0);
  double x1,y1,z1,w1; R1.GetQuaternion(x1,y1,z1,w1);
  tf2::Quaternion q0(x0,y0,z0,w0), q1(x1,y1,z1,w1);
  tf2::Quaternion qi = q0.slerp(q1, s);
  return KDL::Rotation::Quaternion(qi.x(), qi.y(), qi.z(), qi.w());
}

/* Pose -> best IK solution (closest to ref if provided) */
static bool ikFromPose(const geometry_msgs::Pose& pose,
                       std::vector<double>&       q_out,
                       const std::vector<double>* q_ref = nullptr)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  KDL::Rotation R = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
  KDL::Vector   P(pose.position.x, pose.position.y, pose.position.z);

  double T[16] = {
    R(0,0), R(1,0), R(2,0), 0,
    R(0,1), R(1,1), R(2,1), 0,
    R(0,2), R(1,2), R(2,2), 0,
    P.x(),  P.y(),  P.z(),  1
  };

  double sol[8 * 6];
  int nsol = ur_kinematics::inverse(T, sol);
  if (nsol == 0) return false;

  q_out.assign(6, 0.0);
  if (!q_ref) {
    for (int j = 0; j < 6; ++j) q_out[j] = sol[j];
    return true;
  }

  double best = 1e9;
  for (int k = 0; k < nsol; ++k) {
    double d = 0.0;
    for (int j = 0; j < 6; ++j)
      d += std::fabs(sol[k * 6 + j] - (*q_ref)[j]);
    if (d < best) {
      best = d;
      for (int j = 0; j < 6; ++j)
        q_out[j] = sol[k * 6 + j];
    }
  }
  return true;
}

/* ---------------------------------------------------------------------- */
MotionLibrary::MotionLibrary(const KDL::Chain&               /*chain*/,
                             const std::vector<std::string>& names,
                             double                          dt)
: joint_names_(names), dt_(dt)
{}

/* ---------------------------------------------------------------------- */
trajectory_msgs::JointTrajectory
MotionLibrary::generateJointMove(const std::vector<double>& q_curr,
                                 const std::vector<double>& q_start,
                                 const std::vector<double>& q_target,
                                 double v_max, double a_max,
                                 double tolerance_rad) const
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
    seg.header.stamp = ros::Time::now();
    seg.joint_names  = joint_names_;

    for (double t = 0.0; t <= T + 1e-9; t += dt_) {
      double s = clamp01(t / T);
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions.resize(dof);
      for (size_t i = 0; i < dof; ++i)
        pt.positions[i] = qa[i] + s * (qb[i] - qa[i]);
      pt.velocities.resize(dof, 0.0);
      pt.time_from_start = ros::Duration(t);
      seg.points.push_back(pt);
    }
    return seg;
  };

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names_;

  bool withinTol = true;
  for (size_t i = 0; i < dof; ++i)
    if (std::fabs(q_curr[i] - q_start[i]) > tolerance_rad) withinTol = false;

  if (!withinTol) {
    auto seg0 = build_segment(q_curr, q_start);
    traj.points.insert(traj.points.end(),
                       seg0.points.begin(), seg0.points.end());
  }

  auto seg1 = build_segment(q_start, q_target);

  ros::Duration offset = traj.points.empty() ?
                         ros::Duration(0) :
                         traj.points.back().time_from_start + ros::Duration(dt_);
  for (auto& pt : seg1.points) pt.time_from_start += offset;
  if (!traj.points.empty() && !seg1.points.empty())
    seg1.points.erase(seg1.points.begin());

  traj.points.insert(traj.points.end(),
                     seg1.points.begin(), seg1.points.end());
  return traj;
}

trajectory_msgs::JointTrajectory
MotionLibrary::generateLinearMove(const geometry_msgs::Pose& P0,
                                  const geometry_msgs::Pose& P1,
                                  double v_lin,
                                  double a_lin,
                                  const std::vector<double>& q_seed) const
{
  // Convert start/end poses into KDL frames
  tf2::Quaternion q0_tf, q1_tf;
  tf2::fromMsg(P0.orientation, q0_tf);
  tf2::fromMsg(P1.orientation, q1_tf);

  KDL::Frame F0(
    KDL::Rotation::Quaternion(q0_tf.x(), q0_tf.y(), q0_tf.z(), q0_tf.w()),
    KDL::Vector(P0.position.x, P0.position.y, P0.position.z)
  );
  KDL::Frame F1(
    KDL::Rotation::Quaternion(q1_tf.x(), q1_tf.y(), q1_tf.z(), q1_tf.w()),
    KDL::Vector(P1.position.x, P1.position.y, P1.position.z)
  );

  // Compute total motion time via trapezoidal profile
  double dist = (F1.p - F0.p).Norm();
  double T    = profileTime(dist, v_lin, a_lin);

  // Prepare output trajectory
  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now() + ros::Duration(0.1);
  traj.joint_names  = joint_names_;

  // Initialize the IK seed from caller‑provided joint vector
  std::vector<double> q_prev = q_seed;
  if (q_prev.size() != joint_names_.size()) {
    ROS_ERROR("generateLinearMove: seed size mismatch (%zu vs %zu)",
              q_prev.size(), joint_names_.size());
    return trajectory_msgs::JointTrajectory{};
  }

  // Interpolate in Cartesian space, solve IK at each step
  for (double t = 0.0; t <= T + 1e-9; t += dt_) {
    double s = clamp01(t / T);

    // Interpolate position & orientation
    KDL::Vector   P = F0.p + s * (F1.p - F0.p);
    KDL::Rotation R = slerpKDL(F0.M, F1.M, s);
    KDL::Frame    F(R, P);

    // Build transform matrix for IKFast
    double Tm[16] = {
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      P.x(),  P.y(),  P.z(),  1
    };

    // Solve analytic IK
    double sol[8 * 6];
    int nsol = ur_kinematics::inverse(Tm, sol);
    if (nsol == 0) {
      ROS_WARN_STREAM("IKFast: no solution at t=" << t << "; aborting.");
      return trajectory_msgs::JointTrajectory{};
    }

    // Pick the solution closest to the previous one
    std::vector<double> q_sol(6);
    double best_err = std::numeric_limits<double>::infinity();
    for (int k = 0; k < nsol; ++k) {
      double err = 0.0;
      for (size_t j = 0; j < q_prev.size(); ++j) {
        double dq = sol[k*6 + j] - q_prev[j];
        err += std::fabs(dq);
      }
      if (err < best_err) {
        best_err = err;
        for (size_t j = 0; j < q_prev.size(); ++j) {
          q_sol[j] = sol[k*6 + j];
        }
      }
    }
    q_prev = q_sol;

    // Append this point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions       = q_sol;
    pt.velocities.resize(q_sol.size(), 0.0);
    pt.time_from_start = ros::Duration(t) + ros::Duration(0.1);
    traj.points.push_back(pt);
  }

  return traj;
}

#pragma once
#include <vector>

namespace traj {

/** Lightweight trapezoidal‑profile description */
struct Profile {
  double t_acc;   //!< accel time
  double t_flat;  //!< constant‑vel time (0 => triangular)
  double t_total;
  double s_total; //!< distance
};

/** Compute profile parameters for 1‑DOF motion */
Profile generate(double s, double v_max, double a_max);

/** Sample position / velocity at t∈[0,t_total] */
inline void sample(const Profile& p, double t,
                   double& pos_out, double& vel_out);
} // namespace traj

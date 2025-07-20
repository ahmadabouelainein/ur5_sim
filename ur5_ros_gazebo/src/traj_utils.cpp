#include "ur5_ros_gazebo/traj_utils.hpp"
#include <algorithm>
#include <cmath>

namespace traj {
Profile generate(double s, double v_max, double a_max)
{
  Profile p{};
  double t_acc = v_max / a_max;
  double s_acc = 0.5 * a_max * t_acc * t_acc;
  if (2*s_acc > std::fabs(s)) {                       // triangular
    t_acc   = std::sqrt(std::fabs(s)/a_max);
    p.t_acc = t_acc;
    p.t_flat = 0.0;
  } else {                                            // trapezoid
    p.t_acc  = t_acc;
    p.t_flat = (std::fabs(s) - 2*s_acc) / v_max;
  }
  p.t_total = 2*p.t_acc + p.t_flat;
  p.s_total = s;
  return p;
}

void sample(const Profile& p,double t,double& pos,double& vel)
{
  const double a = p.s_total>0 ? 1.0 : -1.0;
  double t1 = p.t_acc, t2 = p.t_acc + p.t_flat;
  if (t < t1) {                       // accel
    vel = a * t * (a*p.s_total>0 ? 1 : -1); // sign maintain
    pos = 0.5 * vel * t;
  } else if (t < t2) {                // cruise
    vel = a * (p.s_total>0 ? 1 : -1);
    pos = a*0.5*t1*t1 + vel*(t - t1);
  } else if (t < p.t_total) {         // decel
    double td = t - t2;
    vel = a*(1 - td);
    pos = a*0.5*t1*t1 + vel*p.t_flat + vel*td - 0.5*td*td;
  } else {                            // done
    vel = 0;
    pos = p.s_total;
  }
}
} // namespace traj

#include <ur5_ros_gazebo/kdl_ik.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ros/console.h>   // ROS_DEBUG / ROS_ERROR

namespace ik
{

// ─────────────────────────────────────────────────────────────────────────────
Solver::Solver(const KDL::Chain&    chain,
               const KDL::JntArray& qmin,
               const KDL::JntArray& qmax)
: chain_(chain)
, qmin_(qmin)
, qmax_(qmax)
{
  if (qmin_.rows() != chain_.getNrOfJoints() ||
      qmax_.rows() != chain_.getNrOfJoints())
  {
    throw std::runtime_error("[kdl_ik] joint‑limit vector size "
                             "does not match chain DOFs");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
bool Solver::solve(const KDL::Frame&          pose,
                   const std::vector<double>& seed,
                   std::vector<double>&       solution) const
{
  const unsigned int dofs = chain_.getNrOfJoints();
  if (seed.size() != dofs)
  {
    ROS_ERROR_STREAM("[kdl_ik] seed size (" << seed.size()
                     << ") ≠ DOFs (" << dofs << ")");
    return false;
  }

  // --- instantiate temporary solver objects (they are stateless) ------------
  KDL::ChainFkSolverPos_recursive fk(chain_);
  KDL::ChainIkSolverVel_pinv      ik_vel(chain_);
  constexpr unsigned max_iter = 200;
  constexpr double   eps      = 1e-4;      // positional / angular tolerance
  KDL::ChainIkSolverPos_NR_JL    ik_pos(chain_, qmin_, qmax_,
                                        fk, ik_vel, max_iter, eps);

  // --- convert seed to KDL array --------------------------------------------
  KDL::JntArray q_seed(dofs);
  for (unsigned i = 0; i < dofs; ++i) q_seed(i) = seed[i];

  KDL::JntArray q_out(dofs);
  int rc = ik_pos.CartToJnt(q_seed, pose, q_out);
  if (rc < 0)
  {
    ROS_DEBUG_STREAM("[kdl_ik] IK failed, error code " << rc);
    return false;
  }

  solution.resize(dofs);
  for (unsigned i = 0; i < dofs; ++i) solution[i] = q_out(i);
  return true;
}

}  // namespace ik

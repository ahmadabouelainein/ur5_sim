#pragma once
// High‑level, stateless IK wrapper around Orocos‑KDL NR_JL solver.

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <vector>

namespace ik
{

class Solver
{
public:
  /** Build a solver for the given kinematic chain and joint‑limit arrays. */
  Solver(const KDL::Chain&      chain,
         const KDL::JntArray&   qmin,
         const KDL::JntArray&   qmax);

  /** Solve IK.
   *
   *  @param pose      Desired end‑effector `KDL::Frame` in the chain base.
   *  @param seed      Initial joint guess (size == dofs).
   *  @param solution  Filled with joint result on success.
   *  @return `true` if a solution was found within error tolerance.
   */
  bool solve(const KDL::Frame&            pose,
             const std::vector<double>&   seed,
             std::vector<double>&         solution) const;

private:
  KDL::Chain     chain_;
  KDL::JntArray  qmin_;
  KDL::JntArray  qmax_;
};

}  // namespace ik

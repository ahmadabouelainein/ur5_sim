#!/usr/bin/env python3
"""
High‑level Python wrapper around the motion‑action server.

Features
========
• Read the latest /joint_states (robot state).
• Send joint‑space or Cartesian (linear) motion requests via actionlib.
• Optional feedback callback prints % complete.
"""

import rospy
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ur5_ros_gazebo.msg import (
    MoveJointAction,  MoveJointGoal,
    MoveLinearAction, MoveLinearGoal
)

class MotionAPI:
    def __init__(self, joint_action_ns='move_joint',
                       linear_action_ns='move_linear'):
        """Connect to action servers and begin listening for joint states."""
        self._state = None
        rospy.Subscriber('/joint_states', JointState,
                         lambda m: setattr(self, '_state', m), queue_size=1)

        self._ac_joint  = actionlib.SimpleActionClient(joint_action_ns,
                                                       MoveJointAction)
        self._ac_linear = actionlib.SimpleActionClient(linear_action_ns,
                                                       MoveLinearAction)

        rospy.loginfo("Waiting for motion action servers...")
        self._ac_joint.wait_for_server()
        self._ac_linear.wait_for_server()
        rospy.loginfo("MotionAPI ready.")

    # ---------------------------------------------------------------------
    # Robot state
    # ---------------------------------------------------------------------
    def get_state(self):
        """Return the last JointState message received (or None)."""
        return self._state

    # ---------------------------------------------------------------------
    # Joint‑space primitive
    # ---------------------------------------------------------------------
    def move_joint(self, q_start, q_target, v_max=1.0, a_max=2.0,
                   wait=True, feedback_cb=None, timeout=None):
        """
        Send a joint‑space motion request.

        Parameters
        ----------
        q_start, q_target : list[float]
            Start and goal joint positions [rad] (same length as robot DOF).
        v_max, a_max : float
            Scalar max velocity/acceleration limits applied to all joints.
        wait : bool
            If True, block until goal completes.  When False, return immediately.
        feedback_cb : callable(feedback_msg)
            Optional callback for MoveJointFeedback messages.
        timeout : rospy.Duration or None
            If given and wait=True, abort wait after this duration.
        """
        goal = MoveJointGoal(q_start=q_start,
                             q_target=q_target,
                             v_max=v_max,
                             a_max=a_max)
        self._ac_joint.send_goal(goal, feedback_cb=feedback_cb)
        if wait:
            if timeout is None:
                finished = self._ac_joint.wait_for_result()
            else:
                finished = self._ac_joint.wait_for_result(timeout)
            return finished and self._ac_joint.get_result().success
        return True

    # ---------------------------------------------------------------------
    # Cartesian linear primitive
    # ---------------------------------------------------------------------
    def move_linear(self, pose_start: Pose, pose_goal: Pose,
                    v_lin=0.10, a_lin=0.25,
                    q_seed=None,
                    wait=True, feedback_cb=None, timeout=None):
        """
        Straight‑line Cartesian motion (see C++ MotionLibrary).

        q_seed : list[float] or None
            Initial IK seed (optional).  Pass [] or None for zeros.
        """
        goal = MoveLinearGoal(pose_start=pose_start,
                              pose_goal=pose_goal,
                              v_lin=v_lin,
                              a_lin=a_lin,
                              q_seed=(q_seed or []))
        self._ac_linear.send_goal(goal, feedback_cb=feedback_cb)
        if wait:
            if timeout is None:
                finished = self._ac_joint.wait_for_result()
            else:
                finished = self._ac_joint.wait_for_result(timeout)
            return finished and self._ac_linear.get_result().success
        return True

# -------------------------------------------------------------------------
#  Helper: move_linear_using_current_state
# -------------------------------------------------------------------------
    def move_linear_using_current_state(self,
                                        pose_start,
                                        pose_goal,
                                        v_lin=0.10,
                                        a_lin=0.25,
                                        wait=True,
                                        timeout=None):
        """
        Convenience wrapper around self.move_linear() that:
        1. Waits until a /joint_states message arrives.
        2. Uses its 'position' array (size == DOF) as the q_seed.
        """
        # 1) fetch current joint state (block a moment if necessary)
        while self.get_state() is None and not rospy.is_shutdown():
            rospy.sleep(0.05)                       # wait for first message

        q_seed = list(self.get_state().position)     # convert tuple→list (6 elems)

        # 2) call the original primitive with the seed
        return self.move_linear(pose_start, pose_goal,
                            v_lin=v_lin, a_lin=a_lin,
                            q_seed=q_seed,
                            wait=wait, timeout=timeout)
# -------------------------------------------------------------------------
# Example usage when run as a script
# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('motion_api_demo')

    api = MotionAPI()
    print(f"Current State:\n{api.get_state()}")
    # 1) Simple joint move demo
    q0 = [0, -1.57, 1.57, 0, 0, 0]
    q1 = [1.57, 1.57, 1.57, 0, 1.57, 1.57]  
    success = api.move_joint(api.get_state().position, q1, feedback_cb=
                             lambda fb: rospy.loginfo(f'Joint motion {fb.percent_complete:.0f}%'))
    rospy.loginfo(f'Joint motion success: {success}')

    # 2) Simple Cartesian line demo
    p0 = Pose()
    p1 = Pose()
    p1.position.x = 0.45
    p1.position.z = 0.45
    # success = move_linear_using_current_state(api, p0, p1,
    #                                       v_lin=0.1, a_lin=0.25)
    # rospy.loginfo(f'Linear motion success: {success}')

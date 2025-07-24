#!/usr/bin/env python3
"""
High‑level Python wrapper around the motion‑action server.

Features
========
• Read the latest /joint_states (robot state).
• Send joint‑space or Cartesian (linear) motion requests via actionlib.
• Optional feedback callback prints % complete.
"""
import tf2_ros
from tf.transformations import quaternion_from_euler
import rospy
import actionlib
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ur5_ros_gazebo.msg import (
    MoveJointAction,  MoveJointGoal,
    MoveLinearAction, MoveLinearGoal
)
import time
CONTROLLER_ORDER = ['shoulder_pan_joint', 'shoulder_lift_joint',
                    'elbow_joint', 'wrist_1_joint',
                    'wrist_2_joint', 'wrist_3_joint']


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
        self.buf  = tf2_ros.Buffer()
        self.lis  = tf2_ros.TransformListener(self.buf)

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

    def _reorder_to_controller(self, joint_state):
        name_to_idx = {n: i for i, n in enumerate(joint_state.name)}
        return [joint_state.position[name_to_idx[n]] for n in CONTROLLER_ORDER]
    # ---------------------------------------------------------------------
    # Joint‑space primitive
    # ---------------------------------------------------------------------
    def move_joint(self, q_start, q_target, v_max=0.20, a_max=0.20,
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
                    v_lin=0.01, a_lin=0.05,
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
                finished = self._ac_linear.wait_for_result()
            else:
                finished = self._ac_linear.wait_for_result(timeout)
            return finished and self._ac_linear.get_result().success
        return True

# -------------------------------------------------------------------------
#  Helper: move_linear_using_current_state
# -------------------------------------------------------------------------
    def move_linear_using_current_state(self,
                                        pose_start,
                                        pose_goal,
                                        v_lin=0.000005,
                                        a_lin=0.00001,
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

        q_seed = self._reorder_to_controller(self.get_state())

        # 2) call the original primitive with the seed
        return self.move_linear(pose_start, pose_goal,
                            v_lin=v_lin, a_lin=a_lin,
                            q_seed=q_seed,
                            wait=wait, timeout=timeout)
        
    def current_pose_from_tf(self):
        trans = self.buf.lookup_transform("base_link", "tool0",
                                    rospy.Time(0), rospy.Duration(0.2))
        pose  = Pose()
        pose.position = trans.transform.translation
        pose.orientation = trans.transform.rotation
        return pose
# -------------------------------------------------------------------------
# Example usage when run as a script
# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('motion_api_demo')

    api = MotionAPI()
    # print(f"Current State:\n{api.get_state()}]n")
    # print()
    # 1) Simple joint move demo
    q0 = [-1.57, -1.57, 1.57, 0, 0, 0]
    q1 = [1.57, 1.57, 1.57, 0, 1.57, 1.57]  
    success = api.move_joint(api.get_state().position, q1, feedback_cb=lambda fb: rospy.loginfo(f'Joint motion {fb.percent_complete:.0f}%'))
    time.sleep(5)
    # 2) Simple Cartesian line demo
    print(api.current_pose_from_tf())
    p0 = api.current_pose_from_tf()
    p0.position.z+=0.1
    p1 = Pose()
    qx, qy, qz, qw = quaternion_from_euler(0, -1.57, 0)  # pitch‑down 90°
    p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w = qx, qy, qz, qw
    p1.position = p0.position
    p1.position.y += 0.15
    p1.position.z -= 0.15
    success = api.move_linear_using_current_state(p0, p1)
    print(api.current_pose_from_tf())

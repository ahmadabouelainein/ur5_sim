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
               wait=True, feedback_cb=None, timeout=None, raise_on_fail=False):
        goal = MoveJointGoal(q_start=q_start,
                            q_target=q_target,
                            v_max=v_max,
                            a_max=a_max)

        if wait:
            # Blocks until done (or until the optional timeouts fire)
            exec_to   = timeout or rospy.Duration(0)   # 0 == forever
            preempt_to = rospy.Duration(0)
            state = self._ac_joint.send_goal_and_wait(goal, exec_to, preempt_to)
            res = self._ac_joint.get_result()
            success = (state == actionlib.GoalStatus.SUCCEEDED) and res and res.success
            if not success and raise_on_fail:
                raise RuntimeError(f"move_joint failed: state={state}, res={res}")
            return success
        else:
            self._ac_joint.send_goal(goal, feedback_cb=feedback_cb)
            return True

    # ---------------------------------------------------------------------
    # Cartesian linear primitive
    # ---------------------------------------------------------------------
    def move_linear(self, pose_start: Pose, pose_goal: Pose,
                    v_lin=0.01, a_lin=0.05,
                    p_seed=None,
                    wait=True, feedback_cb=None, timeout=None):
        """
        Straight‑line Cartesian motion (see C++ MotionLibrary).

        """
        goal = MoveLinearGoal(pose_start=pose_start,
                              pose_goal=pose_goal,
                              v_lin=v_lin,
                              a_lin=a_lin,
                              p_seed=(p_seed or []))
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
        2. Uses its 'position' array (size == DOF) as the p_seed.
        """
        # 1) fetch current joint state (block a moment if necessary)
        while self.get_state() is None and not rospy.is_shutdown():
            rospy.sleep(0.05)                       # wait for first message

        p_seed = self.current_pose_from_tf()

        # 2) call the original primitive with the seed
        return self.move_linear(pose_start, pose_goal,
                            v_lin=v_lin, a_lin=a_lin,
                            p_seed=p_seed,
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
    # print()
    # 1) Simple joint move demo
    q0 = [1.57, -1, 1.57, 0, 1.57, 1.57]  
    q1 = [-1.57, -1, -1.57, 0, 1.57, 1.57]  
    q_start = api._reorder_to_controller(api.get_state())
    success = api.move_joint(q_start, q0, feedback_cb=lambda fb: rospy.loginfo(f'Joint motion {fb.percent_complete:.0f}%'))
    success = api.move_joint(q0, q1, feedback_cb=lambda fb: rospy.loginfo(f'Joint motion {fb.percent_complete:.0f}%'))
    print(f"=============================\nTarget joint state: \n {q1}\nJoint state reached: \n {api._reorder_to_controller(api.get_state())}")
    # time.sleep(5)
    # 2) Simple Cartesian line demo
    p0 = api.current_pose_from_tf()
    # p0.position.z+=0.1
    p1 = Pose()
    qx, qy, qz, qw = quaternion_from_euler(0, -1.57, 0)  # pitch‑down 90°
    p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w = qx, qy, qz, qw
    p1.position = p0.position
    # p1.position.y += 0.15
    p1.position.z += 0.1
    success = api.move_linear_using_current_state(p0, p1)
    print(f"=============================\nTarget cartesian pose: \n {p1}\nCartesian pose reached: \n {api.current_pose_from_tf()}")

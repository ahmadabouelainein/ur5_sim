#!/usr/bin/env python3
import rospy
import actionlib
import tf2_ros
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from ur5_ros_gazebo.msg import (
    MoveJointAction, MoveJointGoal,
    MoveLinearAction, MoveLinearGoal
)

_DEFAULT_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint',
    'elbow_joint', 'wrist_1_joint',
    'wrist_2_joint', 'wrist_3_joint'
]
_JOINT_TOL = 1e-3
_POSE_POS_TOL = 1e-4
_POSE_ORI_TOL = 1e-3

class MotionAPI:
    """
    Python client for UR5 motion actions.

    This class provides synchronous and asynchronous methods to send joint-space
    and Cartesian (linear) motion goals to ROS action servers. It also offers
    sequence helpers that automatically prepend the current robot state when
    needed based on configurable tolerances.
    """

    def __init__(
        self,
        joint_ns: str = 'move_joint',
        linear_ns: str = 'move_linear',
        controller_joints: list = _DEFAULT_JOINTS
    ) -> None:
        """
        Set up ROS subscribers, action clients, and TF listener.

        Args:
            joint_ns: Namespace of the MoveJoint action server.
            linear_ns: Namespace of the MoveLinear action server.
            controller_joints: Joint names in the order expected by the controller.
        """
        self._latest_js = None
        self._joints = controller_joints
        rospy.Subscriber('/joint_states', JointState, self._js_cb, queue_size=1)
        self._ac_joint = actionlib.SimpleActionClient(joint_ns, MoveJointAction)
        self._ac_linear = actionlib.SimpleActionClient(linear_ns, MoveLinearAction)
        self._tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buf)
        self._ac_joint.wait_for_server()
        self._ac_linear.wait_for_server()

    def _js_cb(self, msg: JointState) -> None:
        """
        Callback for /joint_states updates.

        Stores the most recent joint angles for sequence planning and tolerance checks.

        Args:
            msg: The incoming JointState message.
        """
        self._latest_js = msg

    def _get_current_js(self) -> list:
        """
        Retrieve the latest joint positions reordered for the controller.

        Returns:
            A list of joint angles matching the controller's joint ordering.

        Raises:
            RuntimeError: If no joint states are received within the timeout.
        """
        if self._latest_js is None:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            if msg is None:
                raise RuntimeError("No joint state received")
            self._latest_js = msg
        idx = {n: i for i, n in enumerate(self._latest_js.name)}
        return [self._latest_js.position[idx[j]] for j in self._joints]

    def _get_current_pose(self) -> Pose:
        """
        Lookup the current end-effector (tool0) pose in the base_link frame.

        Returns:
            The Pose of the tool frame.

        Raises:
            tf2_ros.LookupException: If the transform is unavailable.
            tf2_ros.ExtrapolationException: If the transform is out of range.
        """
        tf = self._tf_buf.lookup_transform(
            'base_link', 'tool0', rospy.Time(0), rospy.Duration(0.2)
        )
        pose = Pose()
        pose.position = tf.transform.translation
        pose.orientation = tf.transform.rotation
        return pose

    def _is_within_tolerance_joint(
        self, q1: list, q2: list, tol: float = _JOINT_TOL
    ) -> bool:
        """
        Compare two joint configurations within a specified radian tolerance.

        Args:
            q1: First joint vector.
            q2: Second joint vector.
            tol: Maximum allowed absolute difference per joint.

        Returns:
            True if every joint difference is <= tol.
        """
        return all(abs(a - b) <= tol for a, b in zip(q1, q2))

    def _is_within_tolerance_pose(
        self, p1: Pose, p2: Pose,
        pos_tol: float = _POSE_POS_TOL,
        ori_tol: float = _POSE_ORI_TOL
    ) -> bool:
        """
        Compare two poses within position and quaternion tolerances.

        Args:
            p1: First pose.
            p2: Second pose.
            pos_tol: Max Euclidean distance between positions.
            ori_tol: Max Euclidean norm difference between quaternions.

        Returns:
            True if both position and orientation differences are within tolerance.
        """
        dp = ((p1.position.x - p2.position.x)**2 +
              (p1.position.y - p2.position.y)**2 +
              (p1.position.z - p2.position.z)**2)**0.5
        dq = ((p1.orientation.x - p2.orientation.x)**2 +
              (p1.orientation.y - p2.orientation.y)**2 +
              (p1.orientation.z - p2.orientation.z)**2 +
              (p1.orientation.w - p2.orientation.w)**2)**0.5
        return dp <= pos_tol and dq <= ori_tol

    def move_joint(
        self,
        q_start: list,
        q_goal: list,
        v_max: float = 0.2,
        a_max: float = 0.2,
        **kwargs
    ) -> bool:
        """
        Send a single-target joint-space trajectory.

        Args:
            q_start: Starting joint configuration.
            q_goal: Target joint configuration.
            v_max: Scalar velocity limit for all joints.
            a_max: Scalar acceleration limit.
            **kwargs: Passed to the generic send/wait helper.

        Returns:
            True if the action succeeds, False otherwise.
        """
        goal = MoveJointGoal(
            q_start=q_start,
            q_target=q_goal,
            v_max=v_max,
            a_max=a_max
        )
        return self._send_and_wait(self._ac_joint, goal, **kwargs)

    def move_linear(
        self,
        p_start: Pose,
        p_goal: Pose,
        v_lin: float = 0.01,
        a_lin: float = 0.05,
        **kwargs
    ) -> bool:
        """
        Send a single-target Cartesian (linear) trajectory.

        Args:
            p_start: Starting end-effector pose.
            p_goal: Target end-effector pose.
            v_lin: Max linear velocity [m/s].
            a_lin: Max linear acceleration [m/s²].
            **kwargs: Passed to the generic send/wait helper.

        Returns:
            True if the action succeeds, False otherwise.
        """
        goal = MoveLinearGoal(pose_start=p_start,
                      pose_goal=p_goal,
                      v_lin=v_lin,
                      a_lin=a_lin)

        return self._send_and_wait(self._ac_linear, goal, **kwargs)

    def move_joint_sequence(
        self,
        q0: list,
        q1: list,
        v_max: float = 0.2,
        a_max: float = 0.2,
        tol: float = _JOINT_TOL,
        **kwargs
    ) -> bool:
        """
        Execute a two-segment joint trajectory: current->q0->q1.

        If the current state is within tol of q0, skips the first segment.

        Args:
            q0: First waypoint in joint space.
            q1: Final target in joint space.
            v_max: Velocity limit for both segments.
            a_max: Acceleration limit for both segments.
            tol: Tolerance to skip the first segment.
            **kwargs: Passed to move_joint calls.

        Returns:
            True if both required segments succeed.
        """
        q_curr = self._get_current_js()
        seq = []
        if not self._is_within_tolerance_joint(q_curr, q0, tol):
            seq.append((q_curr, q0))
        seq.append((q0, q1))
        for start, goal in seq:
            if not self.move_joint(start, goal, v_max, a_max, **kwargs):
                return False
        return True

    def move_linear_sequence(
        self,
        p0: Pose,
        p1: Pose,
        v_lin: float = 0.01,
        a_lin: float = 0.05,
        pos_tol: float = _POSE_POS_TOL,
        ori_tol: float = _POSE_ORI_TOL,
        **kwargs
    ) -> bool:
        """
        Execute a two-segment linear trajectory: current->p0->p1.

        If the current pose is within tolerances of p0, skips the first segment.

        Args:
            p0: First Cartesian waypoint.
            p1: Final Cartesian target.
            v_lin: Linear velocity limit.
            a_lin: Linear acceleration limit.
            pos_tol: Position tolerance to skip first segment.
            ori_tol: Orientation tolerance to skip first segment.
            **kwargs: Passed to move_linear calls.

        Returns:
            True if both required segments succeed.
        """
        p_curr = self._get_current_pose()
        seq = []
        if not self._is_within_tolerance_pose(p_curr, p0, pos_tol, ori_tol):
            seq.append((p_curr, p0))
        seq.append((p0, p1))
        for start, goal in seq:
            if not self.move_linear(start, goal, v_lin, a_lin, **kwargs):
                return False
        return True

    def _send_and_wait(
        self,
        client: actionlib.SimpleActionClient,
        goal,
        wait: bool = True,
        feedback_cb=None,
        timeout=None,
        raise_on_error: bool = True
    ) -> bool:
        """
        Send a goal and optionally wait for its result.

        Args:
            client: The SimpleActionClient instance.
            goal: The action goal message.
            wait: If True, block until completion.
            feedback_cb: Optional feedback callback.
            timeout: rospy.Duration or None for infinite.
            raise_on_error: If True, raises on failure.

        Returns:
            True if the action succeeded, False otherwise.
        """
        if wait:
            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result(timeout or rospy.Duration(2000))
            state = client.get_state()
            res = client.get_result()
            success = (state == actionlib.GoalStatus.SUCCEEDED and res and res.success)
            if not success and raise_on_error:
                raise RuntimeError(f"Action failed: \nstate={state}, result={res}")
            return success
        client.send_goal(goal, feedback_cb=feedback_cb)
        return True

if __name__ == '__main__':
    rospy.init_node('motion_api_demo')
    api = MotionAPI()
    q0 = [1]*6
    q1 = [0]*6
    api.move_joint_sequence(q0, q1)
    p0 = api._get_current_pose()
    p0.position.z += 0.1
    p1 = Pose()
    qx, qy, qz, qw = quaternion_from_euler(0, 0, 1.57)
    api.move_linear_sequence(p0, p1)

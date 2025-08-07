import sys
import types
from pathlib import Path


# Stub minimal ROS modules so motion_api can be imported without ROS installed
rospy = types.ModuleType("rospy")
rospy.Subscriber = lambda *a, **k: None
rospy.Duration = lambda *a, **k: None
rospy.Time = lambda *a, **k: None
rospy.init_node = lambda *a, **k: None
rospy.is_shutdown = lambda: True
sys.modules["rospy"] = rospy

actionlib = types.ModuleType("actionlib")
actionlib.SimpleActionClient = object
actionlib.GoalStatus = types.SimpleNamespace(SUCCEEDED=3)
sys.modules["actionlib"] = actionlib

tf2_ros = types.ModuleType("tf2_ros")
tf2_ros.Buffer = object
tf2_ros.TransformListener = lambda *a, **k: None
tf2_ros.LookupException = Exception
tf2_ros.ExtrapolationException = Exception
sys.modules["tf2_ros"] = tf2_ros

tf_trans = types.ModuleType("tf.transformations")
tf_trans.quaternion_from_euler = lambda *a, **k: (0, 0, 0, 1)
sys.modules["tf.transformations"] = tf_trans
tf_mod = types.ModuleType("tf")
tf_mod.transformations = tf_trans
sys.modules["tf"] = tf_mod

geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")


class Pose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


geometry_msgs_msg.Pose = Pose
geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs.msg = geometry_msgs_msg
sys.modules["geometry_msgs"] = geometry_msgs
sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class JointState:
    def __init__(self):
        self.name = []
        self.position = []


sensor_msgs_msg.JointState = JointState
sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs.msg = sensor_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

ur_msgs = types.ModuleType("ur5_ros_gazebo.msg")


class MoveJointAction:
    pass


class MoveJointGoal:
    pass


class MoveLinearAction:
    pass


class MoveLinearGoal:
    pass


ur_msgs.MoveJointAction = MoveJointAction
ur_msgs.MoveJointGoal = MoveJointGoal
ur_msgs.MoveLinearAction = MoveLinearAction
ur_msgs.MoveLinearGoal = MoveLinearGoal
ur_pkg = types.ModuleType("ur5_ros_gazebo")
ur_pkg.msg = ur_msgs
sys.modules["ur5_ros_gazebo"] = ur_pkg
sys.modules["ur5_ros_gazebo.msg"] = ur_msgs

sys.path.append(str(Path(__file__).resolve().parents[1] / "scripts"))
from motion_api import MotionAPI  # noqa: E402


def _make_pose(x=0, y=0, z=0, ox=0, oy=0, oz=0, ow=1):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = ox
    p.orientation.y = oy
    p.orientation.z = oz
    p.orientation.w = ow
    return p


def test_is_within_tolerance_joint_success():
    api = MotionAPI.__new__(MotionAPI)
    q1 = [0] * 6
    q2 = [5e-4] * 6
    assert api._is_within_tolerance_joint(q1, q2, tol=1e-3)


def test_is_within_tolerance_joint_failure():
    api = MotionAPI.__new__(MotionAPI)
    q1 = [0] * 6
    q2 = [0, 0, 0, 0, 0, 2e-3]
    assert not api._is_within_tolerance_joint(q1, q2, tol=1e-3)


def test_is_within_tolerance_pose_success():
    api = MotionAPI.__new__(MotionAPI)
    p1 = _make_pose()
    p2 = _make_pose(1e-5, -1e-5, 1e-5, 1e-4, -1e-4, 1e-4)
    assert api._is_within_tolerance_pose(p1, p2, pos_tol=1e-4, ori_tol=1e-3)


def test_is_within_tolerance_pose_failure():
    api = MotionAPI.__new__(MotionAPI)
    p1 = _make_pose()
    p2 = _make_pose(2e-4, 0, 0, 0, 0, 2e-3)
    assert not api._is_within_tolerance_pose(p1, p2, pos_tol=1e-4, ori_tol=1e-3)


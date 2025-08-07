import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

# Patch missing ROS modules
sys.modules.setdefault('rospy', SimpleNamespace())
sys.modules.setdefault(
    'actionlib',
    SimpleNamespace(
        SimpleActionClient=SimpleNamespace,
        GoalStatus=SimpleNamespace(SUCCEEDED=3)
    )
)
sys.modules.setdefault(
    'tf2_ros',
    SimpleNamespace(Buffer=lambda: None, TransformListener=lambda buf: None)
)
tf_module = ModuleType('tf')
tf_module.__path__ = []
transformations_module = ModuleType('tf.transformations')
transformations_module.quaternion_from_euler = (
    lambda *args, **kwargs: (0, 0, 0, 1)
)
sys.modules.setdefault('tf', tf_module)
sys.modules.setdefault('tf.transformations', transformations_module)
sys.modules.setdefault('sensor_msgs.msg', SimpleNamespace(JointState=SimpleNamespace))
sys.modules.setdefault('geometry_msgs.msg', SimpleNamespace(Pose=SimpleNamespace))
msg_mod = SimpleNamespace(
    MoveJointAction=object,
    MoveJointGoal=SimpleNamespace,
    MoveLinearAction=object,
    MoveLinearGoal=SimpleNamespace
)
sys.modules.setdefault('ur5_ros_gazebo.msg', msg_mod)
sys.modules.setdefault('ur5_ros_gazebo', SimpleNamespace(msg=msg_mod))

# Ensure path to scripts for import
sys.path.append(str(Path(__file__).resolve().parent.parent / 'scripts'))
motion_api = importlib.import_module('motion_api')


def make_pose(pos, ori):
    pose = SimpleNamespace()
    pose.position = SimpleNamespace(x=pos[0], y=pos[1], z=pos[2])
    pose.orientation = SimpleNamespace(x=ori[0], y=ori[1], z=ori[2], w=ori[3])
    return pose


def test_is_within_tolerance_joint():
    api = motion_api.MotionAPI.__new__(motion_api.MotionAPI)
    assert api._is_within_tolerance_joint([0, 1], [0, 1 + 1e-4], tol=1e-3)
    assert not api._is_within_tolerance_joint([0, 1], [0, 1 + 1e-2], tol=1e-3)


def test_is_within_tolerance_pose():
    api = motion_api.MotionAPI.__new__(motion_api.MotionAPI)
    p1 = make_pose((0, 0, 0), (0, 0, 0, 1))
    p2 = make_pose((1e-5, 0, 0), (0, 0, 5e-4, 0.999999875))
    assert api._is_within_tolerance_pose(p1, p2, pos_tol=1e-4, ori_tol=1e-3)
    p3 = make_pose((1e-3, 0, 0), (0, 0, 0.01, 0.99995))
    assert not api._is_within_tolerance_pose(p1, p3, pos_tol=1e-4, ori_tol=1e-3)

#!/usr/bin/env python3
import pytest, rospy, actionlib
from ur5_ros_gazebo.msg import MoveJointAction, MoveJointResult
from motion_api import MotionAPI, CONTROLLER_ORDER
from sensor_msgs.msg import JointState

@pytest.fixture(scope="module")
def dummy_action_server():
    """
    Start a fake /move_joint server that immediately succeeds; lets us
    test the client wrapper without spinning up Gazebo.
    """
    result = MoveJointResult(success=True)
    srv = actionlib.SimpleActionServer(
        'move_joint', MoveJointAction,
        execute_cb=lambda goal: srv.set_succeeded(result), auto_start=False)
    srv.start()
    yield srv
    srv.unregister()

def publish_joint_state():
    js = JointState()
    js.name = CONTROLLER_ORDER
    js.position = [0]*6
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1, latch=True)
    rospy.sleep(0.1)            # wait for subscribers
    pub.publish(js)

def test_move_joint_success(dummy_action_server):
    rospy.init_node('test_motion_api', anonymous=True)
    publish_joint_state()

    api = MotionAPI()           # waits for server + joint_state
    q0 = [0]*6
    q1 = [0.1]*6
    assert api.move_joint(q0, q1, v_max=0.5, a_max=1.0)

def test_reorder_helper():
    js = JointState(name=CONTROLLER_ORDER, position=[0,1,2,3,4,5])
    api = MotionAPI.__new__(MotionAPI)          # skip __init__
    assert api._reorder_to_controller(js) == [0,1,2,3,4,5]

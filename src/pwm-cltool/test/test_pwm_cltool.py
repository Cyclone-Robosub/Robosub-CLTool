# tests/test_pwm_cltool.py

import os
import tempfile
import pytest
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32MultiArray, Int64
from pwm_cltool.pwm_cltool import Pwm_Cltool, stop_set, down_set, fwd_set


import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from crs_ros2_interfaces.msg import PwmCmd  # Replace with your actual import


class TestPwmCltoolNode(Node):
    
    def __init__(self):
        super().__init__('TestPwmCltool')
        
        # Initialize data storage
        self.received_pwm_data = None
        self.received_is_timed = None
        self.received_pwm_duration = None
        self.received_is_overriding = None
        self.received_control_mode = None
        self.received_position = None
        self.received_waypoint = None
 
        # Create subscribers
        self.pwm_subscriber = self.create_subscription(
            PwmCmd, 'pwm_cmd_topic', self.pwm_callback, 10)
        self.control_mode_subscriber = self.create_subscription(
            String, 'control_mode_topic', self.control_mode_callback, 10)
        self.position_subscriber = self.create_subscription(
            Float32MultiArray, 'position_topic', self.position_callback, 10)
        self.waypoint_subscriber = self.create_subscription(
            Float32MultiArray, 'waypoint_topic', self.waypoint_callback, 10)
    
    def pwm_callback(self, msg):
        self.received_pwm_data = [
            msg.pwm_flt, 
            msg.pwm_frt, 
            msg.pwm_rlt, 
            msg.pwm_rrt, 
            msg.pwm_flb, 
            msg.pwm_frb, 
            msg.pwm_rlb, 
            msg.pwm_rrb]
        self.received_is_timed = msg.is_timed
        self.received_pwm_duration = msg.duration
        self.received_is_overriding = msg.is_overriding

    def control_mode_callback(self, msg):
        self.received_control_mode = msg.data

    def position_callback(self, msg):
        self.received_position = msg.data

    def waypoint_callback(self, msg):
        self.received_waypoint = msg.data

@pytest.fixture
def cl_tool_test_node():
    """
    Pytest fixture to setup and tear down subscriber node above.

    """

    Cltool = Pwm_Cltool()
    if not rclpy.ok():
        rclpy.init()
    
    test_node = TestPwmCltoolNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    
    test_data = {
        'test_node': test_node,
        'executor': executor,
        'Cltool': Cltool
    }
    
    yield test_data
    
    # Cleanup after test completes
    Cltool.exitCLTool()
    executor.shutdown()
    test_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_untimed_pwm_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']
    
    Cltool.pwm(fwd_set)
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_pwm_data == fwd_set
    assert node.received_is_timed is False
    assert node.received_pwm_duration == -1
    assert node.received_is_overriding is False


def test_timed_pwm_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']

    Cltool.timed_pwm(1000, fwd_set)
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_pwm_data == fwd_set
    assert node.received_is_timed is True
    assert node.received_pwm_duration == 1000
    assert node.received_is_overriding is False


def test_waypoint_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']
    
    waypoint = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    Cltool.waypoint(waypoint)
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_waypoint == pytest.approx(waypoint), ...
    f"Received waypoint: {node.received_waypoint}\n"
    f"Expected waypoint: {waypoint}\n"


def test_position_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']

    position = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    Cltool.position(position)
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_position == pytest.approx(position), ...
    f"Received position: {node.received_position}\n"
    f"Expected position: {position}\n"
    
def test_pid_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']

    Cltool.pid()
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_control_mode == 'PID', f"Received control mode: {node.received_control_mode}\nExpected control mode: PID\n"

def test_manual_function(cl_tool_test_node):
    test_data = cl_tool_test_node
    node = test_data['test_node']
    executor = test_data['executor']
    Cltool = test_data['Cltool']

    Cltool.manual()
    rclpy.spin_once(node, timeout_sec=1.0)
    assert node.received_control_mode == 'FeedForward', f"Received control mode: {node.received_control_mode}\nExpected control mode: feed-forward\n"

#@pytest.fixture
#def cleanup_rclpy():
#    """
#    Ensure we shut down ROS after each test to avoid double-init across tests.
#    """
#    yield
#    # Pwm_Cltool.exitCLTool() already calls rclpy.shutdown(), but just in case:
#    if rclpy.ok():
#        rclpy.shutdown()
#
#@pytest.mark.usefixtures('cleanup_rclpy')
#class TestPwmCltoolIntegration:
#    """
#    Integration-style tests for Pwm_Cltool that spin up real ROS 2 publishers
#    and verify on-topic message delivery.
#    """
#
#    def _make_subscriber(self, msg_type, topic_name, qos=10):
#        """
#        Helper to create a temporary node + subscription that records received msgs.
#        """
#        node = rclpy.create_node('test_subscriber')
#        received = []
#        node.create_subscription(msg_type, topic_name, lambda msg: received.append(msg), qos)
#        return node, received
#
#
#    def test_pwm_command_publishes_correctly(self):
#        """Test that pwm() method publishes the correct PWM command message."""
#        # Use a temp log file so we don't pollute workspace
#        tmpfile = tempfile.NamedTemporaryFile(delete=False)
#        tmpfile.close()
#
#        clt = Pwm_Cltool(log_file=tmpfile.name)
#
#        from crs_ros2_interfaces.msg import PwmCmd
#        sub_node, msgs = self._make_subscriber(PwmCmd, 'pwm_cmd_topic')
#
#        test_pwm_set = [1500, 1600, 1400, 1550, 1450, 1500, 1520, 1480]
#        clt.pwm(test_pwm_set)
#
#        rclpy.spin_once(sub_node, timeout_sec=1.0)
#
#        assert len(msgs) == 1, "Expected exactly one PWM command message"
#        
#        received_msg = msgs[0]
#        assert isinstance(received_msg, PwmCmd)
#        
#        assert received_msg.pwm_flt == test_pwm_set[0]
#        assert received_msg.pwm_frt == test_pwm_set[1]
#        assert received_msg.pwm_rlt == test_pwm_set[2]
#        assert received_msg.pwm_rrt == test_pwm_set[3]
#        assert received_msg.pwm_flb == test_pwm_set[4]
#        assert received_msg.pwm_frb == test_pwm_set[5]
#        assert received_msg.pwm_rlb == test_pwm_set[6]
#        assert received_msg.pwm_rrb == test_pwm_set[7]
#        
#        assert received_msg.is_timed is False
#        assert received_msg.duration == -1
#        assert received_msg.is_overriding is False
#
#        # Cleanup
#        sub_node.destroy_node()
#        clt.exitCLTool()
#        os.unlink(tmpfile.name)
#
#    def test_manual_switch_on_init(self):
#        # Use a temp log file so we don't pollute workspace
#        tmpfile = tempfile.NamedTemporaryFile(delete=False)
#        tmpfile.close()
#
#        clt = Pwm_Cltool(log_file=tmpfile.name)
#
#        # Subscribe to the manual_toggle_switch topic
#        sub_node, msgs = self._make_subscriber(Bool, '/manual_toggle_switch')
#        # Allow background spin thread to publish
#        rclpy.spin_once(sub_node, timeout_sec=1.0)
#
#        assert msgs, "Expected at least one manual switch message on init"
#        assert msgs[0].data is True
#
#        # Cleanup
#        sub_node.destroy_node()
#        clt.exitCLTool()
#        os.unlink(tmpfile.name)

#    @pytest.mark.parametrize('pwm_set', [stop_set, fwd_set])
#    def test_override_publishes_override_array_and_duration(self, pwm_set):
#        clt = Pwm_Cltool(log_file='unused.csv')
#
#        # Create subscribers
#        o_node, o_msgs = self._make_subscriber(Bool,     '/manualOverride')
#        a_node, a_msgs = self._make_subscriber(Int32MultiArray, '/array_Cltool_topic')
#        d_node, d_msgs = self._make_subscriber(Int64,     '/duration_Cltool_topic')
#
#        clt.override(durationMS=321, pwm_set=pwm_set)
#
#        # Spin all three subscribers
#        for node in (o_node, a_node, d_node):
#            rclpy.spin_once(node, timeout_sec=1.0)
#
#        # Assertions
#        assert o_msgs and o_msgs[0].data is True
#        assert a_msgs and list(a_msgs[0].data) == pwm_set
#        assert d_msgs and d_msgs[0].data == 321
#
#        # Cleanup
#        for node in (o_node, a_node, d_node):
#            node.destroy_node()
#        clt.exitCLTool()

#    @pytest.mark.parametrize('scale', [1.0, 0.25, 1.75])
#    def test_pwm_sends_scaled_array_and_negative_duration(self, scale):
#        clt = Pwm_Cltool(log_file='unused.csv')
#
#        # Subscribe to array and duration
#        arr_node, arr_msgs = self._make_subscriber(Int32MultiArray, '/array_Cltool_topic')
#        dur_node, dur_msgs = self._make_subscriber(Int64, '/duration_Cltool_topic')
#
#        clt.pwm(fwd_set, scale=scale)
#
#        rclpy.spin_once(arr_node, timeout_sec=1.0)
#        rclpy.spin_once(dur_node, timeout_sec=1.0)
#
#        assert len(arr_msgs) == 1, "Expected one array message"
#        assert len(dur_msgs) == 1, "Expected one duration message"
#        assert dur_msgs[0].data == -1
#
#        arr_node.destroy_node()
#        dur_node.destroy_node()
#        clt.exitCLTool()

#    def test_exitCLTool_publishes_manual_switch_off(self):
#        clt = Pwm_Cltool(log_file='unused.csv')
#
#        # First, drain the initial ON message
#        drain_node, drained = self._make_subscriber(Bool, '/manual_toggle_switch')
#        rclpy.spin_once(drain_node, timeout_sec=1.0)
#        drained.clear()
#
#        # Now exit and check for OFF
#        clt.exitCLTool()
#        rclpy.spin_once(drain_node, timeout_sec=1.0)
#
#        assert drained and drained[0].data is False
#
#        drain_node.destroy_node()

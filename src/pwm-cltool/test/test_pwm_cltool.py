# tests/test_pwm_cltool.py

import os
import tempfile
import pytest
import rclpy

from std_msgs.msg import Bool, Int32MultiArray, Int64
from pwm_cltool.pwm_cltool import Pwm_Cltool, stop_set, fwd_set

@pytest.fixture
def cleanup_rclpy():
    """
    Ensure we shut down ROS after each test to avoid double-init across tests.
    """
    yield
    # Pwm_Cltool.exitCLTool() already calls rclpy.shutdown(), but just in case:
    if rclpy.ok():
        rclpy.shutdown()

@pytest.mark.usefixtures('cleanup_rclpy')
class TestPwmCltoolIntegration:
    """
    Integration-style tests for Pwm_Cltool that spin up real ROS 2 publishers
    and verify on-topic message delivery.
    """

    def _make_subscriber(self, msg_type, topic_name, qos=10):
        """
        Helper to create a temporary node + subscription that records received msgs.
        """
        node = rclpy.create_node('test_subscriber')
        received = []
        node.create_subscription(msg_type, topic_name, lambda msg: received.append(msg), qos)
        return node, received

    def test_manual_switch_on_init(self):
        # Use a temp log file so we don't pollute workspace
        tmpfile = tempfile.NamedTemporaryFile(delete=False)
        tmpfile.close()

        clt = Pwm_Cltool(log_file=tmpfile.name)

        # Subscribe to the manual_toggle_switch topic
        sub_node, msgs = self._make_subscriber(Bool, '/manual_toggle_switch')
        # Allow background spin thread to publish
        rclpy.spin_once(sub_node, timeout_sec=1.0)

        assert msgs, "Expected at least one manual switch message on init"
        assert msgs[0].data is True

        # Cleanup
        sub_node.destroy_node()
        clt.exitCLTool()
        os.unlink(tmpfile.name)

    @pytest.mark.parametrize('pwm_set', [stop_set, fwd_set])
    def test_override_publishes_override_array_and_duration(self, pwm_set):
        clt = Pwm_Cltool(log_file='unused.csv')

        # Create subscribers
        o_node, o_msgs = self._make_subscriber(Bool,     '/manualOverride')
        a_node, a_msgs = self._make_subscriber(Int32MultiArray, '/array_Cltool_topic')
        d_node, d_msgs = self._make_subscriber(Int64,     '/duration_Cltool_topic')

        clt.override(durationMS=321, pwm_set=pwm_set)

        # Spin all three subscribers
        for node in (o_node, a_node, d_node):
            rclpy.spin_once(node, timeout_sec=1.0)

        # Assertions
        assert o_msgs and o_msgs[0].data is True
        assert a_msgs and list(a_msgs[0].data) == pwm_set
        assert d_msgs and d_msgs[0].data == 321

        # Cleanup
        for node in (o_node, a_node, d_node):
            node.destroy_node()
        clt.exitCLTool()

    @pytest.mark.parametrize('scale', [1.0, 0.25, 1.75])
    def test_pwm_sends_scaled_array_and_negative_duration(self, scale):
        clt = Pwm_Cltool(log_file='unused.csv')

        # Subscribe to array and duration
        arr_node, arr_msgs = self._make_subscriber(Int32MultiArray, '/array_Cltool_topic')
        dur_node, dur_msgs = self._make_subscriber(Int64, '/duration_Cltool_topic')

        clt.pwm(fwd_set, scale=scale)

        rclpy.spin_once(arr_node, timeout_sec=1.0)
        rclpy.spin_once(dur_node, timeout_sec=1.0)

        assert len(arr_msgs) == 1, "Expected one array message"
        assert len(dur_msgs) == 1, "Expected one duration message"
        assert dur_msgs[0].data == -1

        arr_node.destroy_node()
        dur_node.destroy_node()
        clt.exitCLTool()

    def test_exitCLTool_publishes_manual_switch_off(self):
        clt = Pwm_Cltool(log_file='unused.csv')

        # First, drain the initial ON message
        drain_node, drained = self._make_subscriber(Bool, '/manual_toggle_switch')
        rclpy.spin_once(drain_node, timeout_sec=1.0)
        drained.clear()

        # Now exit and check for OFF
        clt.exitCLTool()
        rclpy.spin_once(drain_node, timeout_sec=1.0)

        assert drained and drained[0].data is False

        drain_node.destroy_node()

    def test_scaled_pwm_function(self):
        clt = Pwm_Cltool(log_file='unused.csv')
        # If scale < 1, outputs should be closer to stop_pulse (1500)
        scaled = clt.scaled_pwm(fwd_set, 0.5)
        # Every element should lie midway between original and stop_pulse
        for orig, new in zip(fwd_set, scaled):
            mid = stop_set[0] + (orig - stop_set[0]) * 0.5
            assert abs(new - mid) < 2  # allow rounding

        clt.exitCLTool()

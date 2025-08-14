import pytest
import rclpy
from std_msgs.msg import Int32MultiArray, Int64, Bool

@pytest.mark.usefixtures('ros_init_and_shutdown')
class TestPwmPublisher:
    """
    Verify that each publisher is correctly instantiated
    and that publishing methods actually send on the ROS topics.
    """
    @pytest.mark.parametrize(
        "flag, topic_attr",
        [
            (True, "ManualToggleSwitch"),
            (False, "ManualToggleSwitch"),
        ],
    )
    def test_publish_manual_switch(self, pwm_node, flag, topic_attr):
        received = []
        sub = pwm_node.create_subscription(
            Bool,
            getattr(pwm_node, topic_attr).topic_name,
            lambda msg: received.append(msg),
            10,
        )

        pwm_node.publish_manual_switch(flag)
        rclpy.spin_once(pwm_node, timeout_sec=1.0)

        assert len(received) == 1
        assert isinstance(received[0], Bool)
        assert received[0].data is flag

        pwm_node.destroy_subscription(sub)

    @pytest.mark.parametrize(
        'flag, topic_attr',
        [
            (True,  'ManualOverride'),
            (False, 'ManualOverride'),
        ]
    )
    def test_publish_manual_override(self, pwm_node, flag, topic_attr):
        received = []
        sub = pwm_node.create_subscription(
            Bool,
            getattr(pwm_node, topic_attr).topic_name,
            lambda msg: received.append(msg),
            10,
        )

        pwm_node.publish_manual_override(flag)
        rclpy.spin_once(pwm_node, timeout_sec=1.0)

        assert len(received) == 1
        assert isinstance(received[0], Bool)
        assert received[0].data is flag

        pwm_node.destroy_subscription(sub)

    def test_position_callback(self, pwm_node):

        pwm_node.publish_position([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        rclpy.spin_once(pwm_node, timeout_sec=1.0)
        assert pwm_node.current_position == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

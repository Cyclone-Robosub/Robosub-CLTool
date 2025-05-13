import pytest
import rclpy
from std_msgs.msg import Int32MultiArray, Int64, Bool

@pytest.mark.usefixtures('ros_init_and_shutdown')
class TestPwmPublisher:
    """
    Verify that each publisher is correctly instantiated
    and that publishing methods actually send on the ROS topics.
    """

    def test_node_instantiation(self, pwm_node):
        # Ensure all publishers exist and have correct interface
        pubs = {
            "commandPublisher": (Int32MultiArray, "/array_Cltool_topic"),
            "durationPublisher": (Int64, "/duration_Cltool_topic"),
            "ManualToggleSwitch": (Bool, "/manual_toggle_switch"),
            "ManualOverride": (Bool, "/manualOverride"),
        }

        for attr, (msg_type, topic) in pubs.items():
            pub = getattr(pwm_node, attr, None)
            assert pub is not None, f"{attr} missing"
            assert pub.msg_type is msg_type
            assert pub.topic_name == topic

    @pytest.mark.parametrize(
        'data, msg_type, topic_attr',
        [
            ([0, 1, 2, 3, 4, 5, 6, 7], Int32MultiArray, 'commandPublisher'),
        ]
    )
    def test_publish_array(self, pwm_node, data, msg_type, topic_attr):
        received = []

        # create a subscription to mirror exactly the publisher under test
        sub = pwm_node.create_subscription(
            msg_type,
            getattr(pwm_node, topic_attr).topic_name,
            lambda msg: received.append(msg),
            10,
        )

        # call the method under test
        pwm_node.publish_array(data)

        # spin once to let the message through
        rclpy.spin_once(pwm_node, timeout_sec=1.0)

        assert len(received) == 1
        assert isinstance(received[0], msg_type)
        assert received[0].data == data

        pwm_node.destroy_subscription(sub)

    @pytest.mark.parametrize(
        "value, msg_type, topic_attr",
        [
            (42, Int64, "durationPublisher"),
        ],
    )
    def test_publish_duration(self, pwm_node, value, msg_type, topic_attr):
        received = []
        sub = pwm_node.create_subscription(
            msg_type,
            getattr(pwm_node, topic_attr).topic_name,
            lambda msg: received.append(msg),
            10,
        )

        pwm_node.publish_duration(value)
        rclpy.spin_once(pwm_node, timeout_sec=1.0)

        assert len(received) == 1
        assert isinstance(received[0], msg_type)
        assert received[0].data == value

        pwm_node.destroy_subscription(sub)

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

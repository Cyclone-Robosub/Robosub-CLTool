# tests/test_pwm_publisher.py

import pytest
from std_msgs.msg import Int32MultiArray, Bool, Int64
from rclpy.publisher import Publisher
from pwm_cltool.pwm_publisher import Pwm_Publisher

@pytest.fixture
def node():
    """Return a fresh instance of the ROS2 node under test."""
    return Pwm_Publisher()

def test_node_instantiation(node):
    """Ensure that on init the node has four publishers of the correct types."""
    # Check that attributes exist
    assert hasattr(node, 'commandPublisher')
    assert hasattr(node, 'durationPublisher')
    assert hasattr(node, 'ManualToggleSwitch')
    assert hasattr(node, 'ManualOverride')

    # Check they are Publisher instances
    for attr in ('commandPublisher', 'durationPublisher', 'ManualToggleSwitch', 'ManualOverride'):
        pub = getattr(node, attr)
        assert isinstance(pub, Publisher)

    # Verify topic names and message types
    assert node.commandPublisher.topic_name == 'array_Cltool_topic'
    assert node.commandPublisher.msg_type is Int32MultiArray

    assert node.durationPublisher.topic_name == 'duration_Cltool_topic'
    assert node.durationPublisher.msg_type is Int64

    assert node.ManualToggleSwitch.topic_name == 'manual_toggle_switch'
    assert node.ManualToggleSwitch.msg_type is Bool

    assert node.ManualOverride.topic_name == 'manualOverride'
    assert node.ManualOverride.msg_type is Bool

@pytest.mark.parametrize("pwm_array", [
    [0, 100, 200, 300, 400, 500, 600, 700],
    list(range(8)),  # 0..7
])
def test_publish_array(node, pwm_array, monkeypatch):
    """publish_array should wrap the list in Int32MultiArray and call publish()."""
    published = []
    monkeypatch.setattr(node.commandPublisher, 'publish', lambda msg: published.append(msg))

    node.publish_array(pwm_array)

    # exactly one message was published
    assert len(published) == 1
    msg = published[0]
    assert isinstance(msg, Int32MultiArray)
    assert msg.data == pwm_array

@pytest.mark.parametrize("duration", [0, 5, 123456789])
def test_publish_duration(node, duration, monkeypatch):
    """publish_duration should wrap the int in Int64 and call publish()."""
    published = []
    monkeypatch.setattr(node.durationPublisher, 'publish', lambda msg: published.append(msg))

    node.publish_duration(duration)

    assert len(published) == 1
    msg = published[0]
    assert isinstance(msg, Int64)
    assert msg.data == duration

@pytest.mark.parametrize("flag", [True, False])
def test_publish_manual_switch(node, flag, monkeypatch):
    """publish_manual_switch should wrap the bool in Bool and call publish()."""
    published = []
    monkeypatch.setattr(node.ManualToggleSwitch, 'publish', lambda msg: published.append(msg))

    node.publish_manual_switch(flag)

    assert len(published) == 1
    msg = published[0]
    assert isinstance(msg, Bool)
    assert msg.data is flag

@pytest.mark.parametrize("flag", [True, False])
def test_publish_manual_override(node, flag, monkeypatch):
    """publish_manual_override should wrap the bool in Bool and call publish()."""
    published = []
    monkeypatch.setattr(node.ManualOverride, 'publish', lambda msg: published.append(msg))

    node.publish_manual_override(flag)

    assert len(published) == 1
    msg = published[0]
    assert isinstance(msg, Bool)
    assert msg.data is flag

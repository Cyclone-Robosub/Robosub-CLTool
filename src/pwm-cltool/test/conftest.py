import pytest
import rclpy

from pwm_cltool.pwm_publisher import Pwm_Publisher


@pytest.fixture(scope='session')
def ros_init_and_shutdown():
    """Initialize rclpy once for the whole test session and shutdown when all tests are done."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='function')
def pwm_node():
    """Create a fresh Pwm_Publisher node for each test and tear it down after the test finishes."""
    node = Pwm_Publisher()
    yield node
    node.destroy_node()


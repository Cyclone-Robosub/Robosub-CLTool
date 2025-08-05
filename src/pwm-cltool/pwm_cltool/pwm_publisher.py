from typing import List

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool, Int32MultiArray, String
from crs_ros2_interfaces.msg import PwmCmd


class Pwm_Publisher(Node):
    """
    ROS 2 Node for publishing PWM control and override signals to various topics.

    This node is responsible for sending commands related to the pwm control tool (Cltool), including:
    - PwmCmd Messages which include:
        - PWM signal arrays to control thrusters or actuators.
        - Duration values indicating how long a command should be applied.
    - Boolean toggles for manual mode activation and emergency override.

    Topics Published:
        - 'array_Cltool_topic' (PwmCmd): PWM command array.
        - 'manual_toggle_switch' (Bool): Manual mode toggle signal.
        - 'manualOverride' (Bool): Manual override/emergency stop trigger.
    """

    def __init__(self) -> None:
        """Initialize the node and its publishers."""
        super().__init__('python_cltool_node')
        self.commandPublisher: Publisher = self.create_publisher(
            PwmCmd, 'pwm_cmd_topic', 10
        )

        self.ManualToggleSwitch: Publisher = self.create_publisher(
            Bool, 'manual_toggle_switch', 3
        )
        self.ManualOverride: Publisher = self.create_publisher(
            Bool, 'manualOverride', 4
        )
        self.PositionPublisher: Publisher = self.create_publisher(
            Int32MultiArray, 'position_topic', 10
        )
        self.WaypointPublisher: Publisher = self.create_publisher(
            Int32MultiArray, 'waypoint_topic', 10
        )
        self.ControlModePublisher: Publisher = self.create_publisher(
            String, 'control_mode_topic', 10
        )

    def publish_pwm_cmd(self, pwm_array: List[int], is_timed: bool, pwm_duration: float) -> None:
        """
        Publish a list of PWM values to the 'array_Cltool_topic'.

        Args:
            pwm_array (List[int]): List of PWM values (typically 8 elements for 8 thrusters).
            is_timed (bool): Bool for if supplied pwm values should be used for a specific duration or indefinetely.
            pwm_duration (int): Duration to use the pwm values for. Should be ignored if is_time is true.
        """
        msg = PwmCmd()
        msg.pwm_array = pwm_array
        msg.is_timed = is_timed
        msg.pwm_duration = pwm_duration

        self.commandPublisher.publish(msg)

        # Print the published message for debugging
        print(f'Published PWM array: {msg.pwm_array}')
        print(f'Is timed: {msg.is_timed}')
        print(f'Duration: {msg.pwm_duration}')

    def publish_manual_switch(self, isManualEnabled: bool) -> None:
        """
        Publish a boolean flag to toggle manual control mode.

        Args:
            isManualEnabled (bool): True to enable manual control, False to disable.
        """
        msg = Bool()
        msg.data = isManualEnabled
        self.ManualToggleSwitch.publish(msg)

        if isManualEnabled:
            print('Manual mode enabled')
            self.publish_control_mode('FeedForward')
        else:
            self.publish_control_mode('PID')

    def publish_manual_override(self, isMistakeMade: bool) -> None:
        """
        Publish a manual override signal, typically used for emergency stops or correction.

        Args:
            isMistakeMade (bool): True if an override is needed due to error or fault.
        """
        msg = Bool()
        msg.data = isMistakeMade
        self.ManualOverride.publish(msg)

        if isMistakeMade:
            print('Manual override triggered')
        else:
            print('Manual override cleared')

    def publish_position(self, position: List[int]) -> None:
        """
        Publish a list of positions to the 'position_topic'.

        Args:
            position (List[int]): List of positions to publish.
        """
        assert len(position) == 6
        msg = Int32MultiArray()
        msg.data = position
        self.PositionPublisher.publish(msg)

    def publish_waypoint(self, waypoint: List[int]) -> None:
        """
        Publish a list of waypoints to the 'waypoint_topic'.

        Args:
            waypoint (List[int]): List of waypoints to publish.
        """
        assert len(waypoint) == 6
        msg = Int32MultiArray()
        msg.data = waypoint
        self.WaypointPublisher.publish(msg)

    def publish_control_mode(self, control_mode: str) -> None:
        """
        Publish a control mode to the 'control_mode_topic'.

        Args:
            control_mode (str): Control mode to publish.
        """
        msg = String()
        msg.data = control_mode
        self.ControlModePublisher.publish(msg)
        print(msg.data)

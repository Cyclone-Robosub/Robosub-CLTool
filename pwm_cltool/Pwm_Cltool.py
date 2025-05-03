from std_msgs.msg import Int32MultiArray, Bool, Int64
from rclpy.node import Node
from rclpy.publisher import Publisher
from typing import List

class Pwm_Cltool(Node):
    """
    ROS 2 Node for publishing PWM control and override signals to various topics.

    This node is responsible for sending commands related to the pwm control tool (Cltool), including:
    - PWM signal arrays to control thrusters or actuators.
    - Duration values indicating how long a command should be applied.
    - Boolean toggles for manual mode activation and emergency override.

    Topics Published:
        - 'array_Cltool_topic' (Int32MultiArray): PWM command array.
        - 'duration_Cltool_topic' (Int64): Duration of the command in seconds.
        - 'manual_toggle_switch' (Bool): Manual mode toggle signal.
        - 'manualOverride' (Bool): Manual override/emergency stop trigger.
    """

    def __init__(self) -> None:
        """
        Initializes the node and its publishers.
        """
        super().__init__("python_cltool_node")
        self.commandPublisher: Publisher = self.create_publisher(
            Int32MultiArray, "array_Cltool_topic", 10
        )
        self.durationPublisher: Publisher = self.create_publisher(
            Int64, "duration_Cltool_topic", 10
        )
        self.ManualToggleSwitch: Publisher = self.create_publisher(
            Bool, "manual_toggle_switch", 3
        )
        self.ManualOverride: Publisher = self.create_publisher(
            Bool, "manualOverride", 4
        )

    def publish_array(self, pwm_array: List[int]) -> None:
        """
        Publishes a list of PWM values to the 'array_Cltool_topic'.

        Args:
            pwm_array (List[int]): List of PWM values (typically 8 elements for 8 thrusters).
        """
        msg = Int32MultiArray()
        msg.data = pwm_array
        self.commandPublisher.publish(msg)
        print(msg.data)

    def publish_duration(self, durationSec: int) -> None:
        """
        Publishes the command duration to the 'duration_Cltool_topic'.

        Args:
            durationSec (int): Duration in seconds for which the PWM command should be applied.
        """
        msg = Int64()
        msg.data = durationSec
        self.durationPublisher.publish(msg)
        print(msg.data)

    def publish_manual_switch(self, isManualEnabled: bool) -> None:
        """
        Publishes a boolean flag to toggle manual control mode.

        Args:
            isManualEnabled (bool): True to enable manual control, False to disable.
        """
        msg = Bool()
        msg.data = isManualEnabled
        self.ManualToggleSwitch.publish(msg)

    def publish_manual_override(self, isMistakeMade: bool) -> None:
        """
        Publishes a manual override signal, typically used for emergency stops or correction.

        Args:
            isMistakeMade (bool): True if an override is needed due to error or fault.
        """
        msg = Bool()
        msg.data = isMistakeMade
        self.ManualOverride.publish(msg)
        print(msg.data)

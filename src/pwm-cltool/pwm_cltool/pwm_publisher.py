from std_msgs.msg import Int32MultiArray, Bool, Int64
from rclpy.node import Node
from rclpy.publisher import Publisher
from typing import List

from crs_ros2_interfaces.msg import PwmCmd

class Pwm_Publisher(Node):
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
            PwmCmd, "pwm_cmd_topic", 10
        )
        self.ManualToggleSwitch: Publisher = self.create_publisher(
            Bool, "manual_toggle_switch", 3
        )
        self.ManualOverride: Publisher = self.create_publisher(
            Bool, "manualOverride", 4
        )

    def publish_pwm_cmd(self, pwm_array: List[int], is_timed: bool, pwm_duration: float) -> None:
        """
        Publishes a list of PWM values to the 'array_Cltool_topic'.

        Args:
            pwm_array (List[int]): List of PWM values (typically 8 elements for 8 thrusters).
        """
        msg = PwmCmd()
        
        msg.pwm_flt = pwm_array[0]
        msg.pwm_frt = pwm_array[1]
        msg.pwm_rlt = pwm_array[2]
        msg.pwm_rrt = pwm_array[3]
        msg.pwm_flb = pwm_array[4]
        msg.pwm_frb = pwm_array[5]
        msg.pwm_rlb = pwm_array[6]
        msg.pwm_rrb = pwm_array[7]
        
        msg.is_timed = is_timed
        
        msg.duration = pwm_duration
        
        self.commandPublisher.publish(msg)
        
        print(
            f"pwm_flt = {msg.pwm_flt}\n"
            f"pwm_frt = {msg.pwm_frt}\n"
            f"pwm_rlt = {msg.pwm_rlt}\n"
            f"pwm_rrt = {msg.pwm_rrt}\n"
            f"pwm_flb = {msg.pwm_flb}\n"
            f"pwm_frb = {msg.pwm_frb}\n"
            f"pwm_rlb = {msg.pwm_rlb}\n"
            f"pwm_rrb = {msg.pwm_rrb}"
        )
        print(f"is_timed = {msg.is_timed}")
        print(f"duration = {msg.duration}")

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

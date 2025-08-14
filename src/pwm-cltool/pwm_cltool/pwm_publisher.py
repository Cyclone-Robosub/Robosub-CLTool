from typing import List

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray, String
from crs_ros2_interfaces.msg import PwmCmd
from rclpy.timer import Timer


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
            Float32MultiArray, 'position_topic', 10
        )
        self.WaypointPublisher: Publisher = self.create_publisher(
            Float32MultiArray, 'waypoint_topic', 10
        )
        self.ControlModePublisher: Publisher = self.create_publisher(
            String, 'control_mode_topic', 10
        )
        self.PwmLimitPublisher: Publisher = self.create_publisher(
            Int32MultiArray, 'pwm_limit_topic', 10
        )
        self.PidGainPublisher: Publisher = self.create_publisher(
            Float32MultiArray, 'pid_gain_topic', 10
        )
        


        self.PositionSubscriber: Subscription = self.create_subscription(
            Float32MultiArray, 'position_topic', self.position_callback, 10
        )
        self.WaypointSubscriber: Subscription = self.create_subscription(
            Float32MultiArray, 'waypoint_topic', self.waypoint_callback, 10
        )
        
        # Periodic timer (default 10 Hz)
        self.timer_period_seconds: float = 0.1
        self._timer: Timer = self.create_timer(self.timer_period_seconds, self.timer_callback)

        self.correction_axis = -1
        self.p_values = [1,1,1,1,1,1]
        self.limits = [1, 1, 1, 1, 1, 1]
        self.current_position: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_waypoint: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.error: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.correction_sets: List[List[int]] = [
            [1440, 1560, 1460, 1540, 1600, 1400, 1600, 1400],
            [1500, 1500, 1500, 1500, 1900, 1900, 1100, 1100],
            [1900, 1100, 1900, 1100, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1900, 1900, 1900, 1900]            
        ]
    
    def timer_callback(self) -> None:

        if self.correction_axis == -1:
            pass
        else:
            self.correct_position(self.correction_axis)
            

    def correct_position(self, axis: int) -> None:
        error = self.current_waypoint[axis] - self.current_position[axis]
        response = self.p_values[axis] * error
        if response > self.limits[axis]:
            response = self.limits[axis]
        elif response < -self.limits[axis]:
            response = -self.limits[axis]

        
        pwm = self.correction_sets[axis]
        pwm = self.scaled_pwm(pwm, response)
        self.publish_pwm_cmd(pwm, False, -1.0, False)

    def scaled_pwm(self, pwm_set: List[int], scale: float) -> List[int]:
        stop_pulse = 1500  
        new_pwm = [int(scale * (i - stop_pulse) + stop_pulse) for i in pwm_set]
        return new_pwm

    def state_error(self) -> None:
        for i in range(6):
            self.error[i] = self.current_position[i] - self.current_waypoint[i]

    def waypoint_callback(self, msg: Float32MultiArray) -> None:
        self.current_waypoint = list(msg.data)

    def position_callback(self, msg: Float32MultiArray) -> None:
        self.current_position = list(msg.data)

    def publish_pwm_limit(self, min: int, max: int) -> None:
        """
        Publish a list of PWM limits to the 'pwm_limit_topic'.
        """
        msg = Int32MultiArray()
        msg.data = [min, max]
        self.PwmLimitPublisher.publish(msg)

    def publish_pwm_cmd(self, pwm_array: List[int], is_timed: bool, pwm_duration: float, is_override: bool) -> None:
        """
        Publish a list of PWM values to the 'array_Cltool_topic'.

        Args:
            pwm_array (List[int]): List of PWM values (typically 8 elements for 8 thrusters).
            is_timed (bool): Bool for if supplied pwm values should be used for a specific duration or indefinetely.
            pwm_duration (int): Duration to use the pwm values for. Should be ignored if is_time is true.
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
        msg.is_overriding = is_override

        self.commandPublisher.publish(msg)

        # Print the published message for debugging
        print(f'Published PWM array: {pwm_array}')
        print(f'Is timed: {is_timed}')
        print(f'Duration: {pwm_duration}')

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

    def publish_position(self, position: List[float]) -> None:
        """
        Publish a list of positions to the 'position_topic'.

        Args:
            position (List[float]): List of positions to publish.
        """
        assert len(position) == 6
        msg = Float32MultiArray()
        msg.data = position
        self.PositionPublisher.publish(msg)

    def _handle_position(self, msg: Float32MultiArray) -> None:
        self.current_position = list(msg.data)

    def publish_waypoint(self, waypoint: List[float]) -> None:
        """
        Publish a list of waypoints to the 'waypoint_topic'.

        Args:
            waypoint (List[float]): List of waypoints to publish.
        """
        assert len(waypoint) == 6
        msg = Float32MultiArray()
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

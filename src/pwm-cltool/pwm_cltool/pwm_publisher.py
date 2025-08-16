from typing import List

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray, String
from crs_ros2_interfaces.msg import PwmCmd
from rclpy.timer import Timer
from math import atan2, sqrt, degrees

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
        super().__init__('python_cltool_node')
        self.commandPublisher: Publisher = self.create_publisher(
            PwmCmd, 'pwm_cmd_topic', 10
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


        self.correction_axis = 0
        self.correction_index = 0
        self.auto_correction_active = False
        self.correction_active = False

        self.p_values = [2,1,0.75,0.005,0.005,0.5]
        self.i_values = [0.03, 0.05, 0.005, 0.0005, 0.0005, 0.05]
        self.d_values = [4, 0.5, 0.5, 0.5, 0.005, 0.05]
        self.i_max = [1000, 1000, 1000, 100, 100, 100]
        self.limits = [0.5, 0.5, 0.5, 1, 1, 0.25]
        self.lower_tolerances = [0.05, 0.05, 0.05, 5, 5, 5]
        self.upper_tolerances = [0.1, 0.1, 0.1, 10, 10, 10]
        self.hold_time = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        self.hold_start_time = 0
        self.hold_flag = False

        # if true, we will correct yaw by world frame error for testing purposes
        self.world_frame_testing = False

        # only correct on z, yaw, and x axis
        self.axis_priority = [2, 5, 0]

        self.current_position: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_waypoint: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.world_error: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.position_error : List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.integral_error: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.derivative_error: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.correction_sets: List[List[int]] = [
            [1440, 1500, 1460, 1540, 1400, 1600, 1500, 1560],
            [1500, 1500, 1500, 1500, 1900, 1100, 1900, 1100],
            [1900, 1100, 1900, 1100, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
            [1500, 1500, 1500, 1500, 1900, 1900, 1900, 1900]            
        ] 

        self.next_pwm_set: List[int] = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]


    def timer_callback(self) -> None:
        self.compute_error()
        if not self.correction_active:
            return
        elif self.auto_correction_active:
            self.auto_correction()
        else:
            self.correct_position(self.correction_axis)

        

    def auto_correction(self) -> None:
        
        # z axis correction always runs
        self.correct_z_axis()

        # x axis correction runs only if yaw within tolerance
        if self.position_error[5] < self.lower_tolerances[5]:
            self.correct_x_axis()

        # otherwise, we will correct yaw
        else:
            self.correct_yaw_axis()

        # publish the next pwm set
        self.publish_pwm_cmd(self.next_pwm_set, False, -1.0, False)
  
    def compute_error(self) -> None:
        print(f"current_position: {self.current_position}, current_waypoint: {self.current_waypoint}")
        for i in range(6):
            self.world_error[i] = self.current_position[i] - self.current_waypoint[i]
            self.compute_body_error(i)
    
    def compute_body_error(self, axis: int) -> None:

        old_error = self.position_error[axis]
        error = 0

        if self.world_frame_testing:
            error = self.world_error[axis]

        # axis 0 -> x axis correction (body frame)
        elif axis == 0:
            error = sqrt(self.world_error[0]**2 + self.world_error[1]**2)

        # axis 1 -> y axis correction (body frame)
        elif axis == 1:
            pass

        # axis 2 -> z axis correction (body frame)
        elif axis == 2:
            error = self.world_error[2]

        # axis 3 -> roll correction (body frame)
        elif axis == 3:
            pass

        # axis 4 -> pitch correction (body frame)
        elif axis == 4:
            pass

        # axis 5 -> yaw correction
        elif axis == 5:
            dx = self.world_error[0]
            dy = self.world_error[1]
            error = atan2(dy, dx)
            yaw_desired = degrees(atan2(dy, dx))
            yaw_current = self.current_position[5]
            error = yaw_desired - yaw_current
            
            while error > 180:
                error -= 360
            while error < -180:
                error += 360
        
        
        self.position_error[axis] = error
        self.derivative_error[axis] = error - old_error
        self.integral_error[axis] += error

        # windup protection
        if self.integral_error[axis] > self.i_max[axis]:
            self.integral_error[axis] = self.i_max[axis]
        elif self.integral_error[axis] < -self.i_max[axis]:
            self.integral_error[axis] = -self.i_max[axis]
            
    def compute_response(self, axis: int) -> None:
        response = self.p_values[axis] * self.position_error[axis]
        response += self.i_values[axis] * self.integral_error[axis]
        response += self.d_values[axis] * self.derivative_error[axis]

        if response > self.limits[axis]:
            response = self.limits[axis]
        elif response < -self.limits[axis]:
            response = -self.limits[axis]

        return response

    def correct_position(self, axis: int) -> None:
        response = self.compute_response(axis)
        pwm = self.correction_sets[axis]
        pwm = self.scaled_pwm(pwm, response)
        self.publish_pwm_cmd(pwm, False, -1.0, False)

    def correct_z_axis(self) -> None:
        response = self.compute_response(2)
        pwm = self.correction_sets[2]
        pwm = self.scaled_pwm(pwm, response)
        active_thrusters = [1, 1, 1, 1, 0, 0, 0, 0]
        for i in range(8):
            if active_thrusters[i]:
                self.next_pwm_set[i] = pwm[i]

    def correct_x_axis(self) -> None:
        response = self.compute_response(0)
        pwm = self.correction_sets[0]
        pwm = self.scaled_pwm(pwm, response)
        active_thrusters = [0, 0, 0, 0, 1, 1, 1, 1]
        for i in range(8):
            if active_thrusters[i]:
                self.next_pwm_set[i] = pwm[i]

    def correct_yaw_axis(self) -> None:
        response = self.compute_response(5)
        pwm = self.correction_sets[5]
        pwm = self.scaled_pwm(pwm, response)
        active_thrusters = [0, 0, 0, 0, 1, 1, 1, 1]
        for i in range(8):
            if active_thrusters[i]:
                self.next_pwm_set[i] = pwm[i]

    def scaled_pwm(self, pwm_set: List[int], scale: float) -> List[int]:
        stop_pulse = 1500  
        new_pwm = [int(scale * (i - stop_pulse) + stop_pulse) for i in pwm_set]
        return new_pwm

    def world_error(self) -> None:
        for i in range(6):
            self.world_error[i] = self.current_position[i] - self.current_waypoint[i]

    def reset_integral_error(self) -> None:
        for i in range(6):
            self.integral_error[i] = 0

    def waypoint_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 6:
            self.current_waypoint = list(msg.data)
        else:
            print(f"Invalid waypoint message received: {msg.data}")

    def position_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 6:
            self.current_position = list(msg.data)
        else:
            print(f"Invalid position message received: {msg.data}")

    def publish_pwm_limit(self, min: int, max: int) -> None:
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




#       OLD PID CORRECTION CODE
#        self.correction_axis = self.axis_priority[self.correction_index]
#
#        # this block will discover if a higher priority axis is out of tolerance, 
#        # if so, we will correct that axis first
#        for i, axis in enumerate(self.axis_priority):
#            if i == self.correction_index:
#                break
#            elif self.position_error[axis] > self.upper_tolerances[axis]:
#                self.correction_index = i
#                self.correction_axis = axis
#                break
#        
#        # this block corrects the highest priority axis that is out of tolerance
#        axis = self.correction_axis
#        if self.position_error[axis] > self.lower_tolerances[axis]:
#            self.hold_flag = False
#            self.correct_position(axis)
#
#        # when the axis is within tolerance, we will hold the position for a short duration
#        elif self.position_error[axis] < self.lower_tolerances[axis] and not self.hold_flag:
#            self.hold_flag = True
#            self.hold_start_time = self.get_clock().now().to_msg().sec
#            self.correct_position(axis)
#
#        # and then increment the correction index to move to the next axis
#        elif self.position_error[axis] < self.lower_tolerances[axis] and self.hold_flag:
#            if self.get_clock().now().to_msg().sec - self.hold_start_time > self.hold_time[axis]:
#                self.hold_flag = False
#                self.correction_index += 1
#            else:
#                self.correct_position(axis)
# 
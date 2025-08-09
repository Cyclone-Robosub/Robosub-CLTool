import threading
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException
import rclpy
from .pwm_publisher import Pwm_Publisher
from .plant import Plant

from time import sleep
from typing import List
import code

import readline
import rlcompleter

# Define the CLTool Globals
rev_pulse: int = 1100
stop_pulse: int = 1500
fwd_pulse_raw: int = 1900
rev_adj: float = 1.0
fwd_pulse: int = int(fwd_pulse_raw * rev_adj)

zero_set: List[int] = [0 for _ in range(8)]
stop_set: List[int] = [stop_pulse for _ in range(8)]

test_sets : List[List[int]] = [ [1500 for _ in range(8)] for _ in range(8) ]
for i in range(8):
    test_sets[i][i] = 1400


fwd_set: List[int] = [stop_pulse for _ in range(4)] + [
    fwd_pulse,
    rev_pulse,
    fwd_pulse,
    rev_pulse,
]
crab_set: List[int] = [stop_pulse for _ in range(4)] + [
    fwd_pulse,
    fwd_pulse,
    rev_pulse,
    rev_pulse,
]
down_set: List[int] = [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse] + [
    stop_pulse for _ in range(4)
]
test_set: List[int] = [1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536]
barrel: List[int] = [fwd_pulse for _ in range(4)] + [stop_pulse for _ in range(4)]
summer: List[int] = [rev_pulse, fwd_pulse, fwd_pulse, rev_pulse] + [
    stop_pulse for _ in range(4)
]
spin_set: List[int] = [stop_pulse for _ in range(4)] + [fwd_pulse for _ in range(4)]

torpedo: List[int] = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [
    fwd_pulse,
    rev_pulse,
    fwd_pulse,
    rev_pulse,
]


class Pwm_Cltool:
    """
    Provides a manual control interface for a thrust system using the Pwm_Cltool ROS 2 node.

    This class initializes publishers for sending PWM signals and duration commands
    to control a robotic system's thrusters. It supports basic manual operations,
    overrides, and signal scaling for real-time testing or remote control.

    Attributes:
        plant (Plant): An instance of the Plant class for modeling/control feedback.
        thrusters (List[int]): Pin numbers for each thruster.
        publishCommandDurationObject (Pwm_Cltool): ROS node used to publish control messages.
        ros_thread (threading.Thread): Thread to keep the ROS node spinning.
    """

    def __init__(self, log_file: str = "pwm_file.csv"):
        """
        Initializes ROS, thruster pins, and sets up publishers in a background thread.
        Automatically enables manual mode and provides CLI instructions.
        """
        self.plant = Plant()
        self.thrusters: List[int] = [3, 2, 5, 4, 18, 20, 19, 21]
        rclpy.init()
        self.publishCommandDurationObject = Pwm_Publisher()
        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.start()
        self.logFile = log_file
        print("Switching onto manual control...")
        sleep(0.5)
        self.publishCommandDurationObject.publish_manual_switch(True)
        print("Ready to input manual commands")
        print("Please type clt.exitCLTool() to safely exit manual control.\n")

    def start_console(self):
        
        locals = {
                "clt": self,
                "stop_set": stop_set,
                "fwd_set": fwd_set,
                "crab_set": crab_set,
                "down_set": down_set,
                "test_sets": test_sets,
                "barrel": barrel,
                "summer": summer,
                "spin_set": spin_set,
                "torpedo": torpedo,
        }
                                                    
        readline.set_completer(rlcompleter.Completer(locals).complete)
        readline.parse_and_bind("tab: complete")
        code.InteractiveConsole(locals).interact()
        
        # Launch an interactive console with `tcs` in the namespace
        banner = (
            "Interactive Thrust_Control Console\n"
            "Available object: clt (Pwm_Cltool instance)\n"
            "Type 'shutdown' to cleanly exit, or use exit()/Ctrl-D.\n"
        )
        console = code.InteractiveConsole(
            locals = locals
        )
        console.interact(banner=banner, exitmsg="Console exiting, shutting down...")

    def override(self, durationMS: float = -1.0, pwm_set: List[int] = stop_set):
        """
        Publishes a manual override command, typically for emergency situations.

        Args:
            durationMS (int): Duration in milliseconds. Use -1 for continuous command.
            pwm_set (List[int]): PWM values to apply to thrusters during override.
        """
        self.publishCommandDurationObject.publish_manual_override(True)
        if durationMS < 0.0:
            is_timed = False
        else:
            is_timed = True
            
        sleep(0.2)
        
        self.publishCommandDurationObject.publish_pwm_cmd(pwm_set, is_timed, durationMS)

    def exitCLTool(self) -> None:
        """
        Safely exits manual control by disabling the manual switch and shutting down ROS.
        """
        print("Shutting down CL Tool and Manual Control")
        self.publishCommandDurationObject.publish_manual_switch(False)
        rclpy.shutdown()

    
    def spin_ros(self) -> None:
        """Spin via manual executor loop to avoid wait-set bugs (e.g. in testing)."""
        executor = SingleThreadedExecutor()
        executor.add_node(self.publishCommandDurationObject)
        # Loop until shutdown is requested
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                # Clean shutdown requested: exit loop
                break

    def pwm(self, pwm_set: List[int], scale: float = 1.0) -> None:
        """
        Sends a scaled or raw PWM command to all thrusters. To send overriding command, use the override command.

        Args:
            pwm_set (List[int]): List of PWM values to apply.
            scale (float): Optional scale factor to apply to the PWM signal.
        """
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
        if len(pwm_set) != len(self.thrusters):
            print("Wrong length for pwm set\n")
            return
        print("pwm function executed.")
        pwm_set = [int(i) for i in pwm_set]
        self.publishCommandDurationObject.publish_pwm_cmd(pwm_set, False, -1, False)

    def scaled_pwm(self, pwm_set: List[int], scale: float) -> List[int]:
        """
        Scales the input PWM values relative to the stop pulse.

        Args:
            pwm_set (List[int]): The original PWM values.
            scale (float): A scaling factor to apply to the PWM signal.

        Returns:
            List[int]: The scaled PWM values.
        """
        new_pwm = [int(scale * (i - stop_pulse) + stop_pulse) for i in pwm_set]
        return new_pwm

    def timed_pwm(self, time_s: float, pwm_set: List[int], scale: float = 1.0) -> None:
        """
        Sends a PWM command for a fixed duration.

        Args:
            time_s (float): Duration in seconds for which to apply the PWM.
            pwm_set (List[int]): PWM values to apply.
            scale (float): Optional scale factor for PWM values.
        """
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
        print('Executing timed_pwm...')
        self.publishCommandDurationObject.publish_pwm_cmd(pwm_set, True, time_s, False)

    def read(self) -> None:
        """Read and print the contents of the PWM command log file."""
        with open(self.logFile, 'r') as logf:
            file_contents = logf.read()
            print(file_contents)

    def reaction(self, pwm_set: List[int], scale: float = 1.0) -> None:
        """
        Send scaled PWM values to the plant model for reaction force estimation.

        Args:
            pwm_set (List[int]): PWM command values.
            scale (float): Optional scale factor to modify PWM before processing.
        """
        pwm = [scale * (i - stop_pulse) + stop_pulse for i in pwm_set]
        self.plant.pwm_force(pwm)


    def waypoint(self, waypoint: List[float]) -> None:
        """
        Sends a waypoint over ROS

        Args:
            waypoint (List[float]): A list of 6 floats representing the waypoint.
        """
        if len(waypoint) != 6:
            print("Wrong length for waypoint\n")
            return
        print(f'Executing waypoint {waypoint}')
        self.publishCommandDurationObject.publish_waypoint(waypoint)

    def position(self, position: List[float]) -> None:
        """
        Sends a position over ROS

        Args:
            position (List[float]): A list of 6 floats representing the position.
            world frame: x, y, z, roll, pitch, yaw, in meters and radians
        """
        if len(position) != 6:
            print("Wrong length for position\n")
            return
        print(f'Executing position {position}')
        self.publishCommandDurationObject.publish_position(position)

    def pid(self) -> None:
        """
        Sets control mode to PID
        """
        self.publishCommandDurationObject.publish_control_mode('PID')

    def manual(self) -> None:
        """
        Sets control mode to manual
        """
        self.publishCommandDurationObject.publish_control_mode('FeedForward')

    def test_thruster(self, thruster_num: int) -> None:
        """
        Tests a single thruster by applying a pulse to it for 1 second.

        Args:
            thruster_num (int): The number of the thruster to test.
        """
        thruster_test_set = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        thruster_test_set[thruster_num] = 1400

        self.timed_pwm(1, thruster_test_set)

    def test_all_thrusters(self) -> None:
        """
        Tests all thrusters by applying a pulse to each one for 1 second.
        """
        for i in range(len(test_sets)):
            print(f'Testing thruster {i}')
            self.test_thruster(i)
            self.timed_pwm(3, stop_set)





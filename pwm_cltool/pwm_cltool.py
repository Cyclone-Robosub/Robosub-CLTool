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
        # Launch an interactive console with `tcs` in the namespace
        readline.parse_and_bind("tab: complete")
        banner = (
            "Interactive Thrust_Control Console\n"
            "Available object: clt (Pwm_Cltool instance)\n"
            "Type 'shutdown' to cleanly exit, or use exit()/Ctrl-D.\n"
        )
        console = code.InteractiveConsole(
            locals={
                "clt": self,
                "stop_set": stop_set,
                "fwd_set": fwd_set,
                "crab_set": crab_set,
                "down_set": down_set,
                "test_set": test_set,
                "barrel": barrel,
                "summer": summer,
                "spin_set": spin_set,
                "torpedo": torpedo,
            }
        )
        console.interact(banner=banner, exitmsg="Console exiting, shutting down...")

    def override(self, durationMS: int = -1, pwm_set: List[int] = stop_set):
        """
        Publishes a manual override command, typically for emergency situations.

        Args:
            durationMS (int): Duration in milliseconds. Use -1 for continuous command.
            pwm_set (List[int]): PWM values to apply to thrusters during override.
        """
        self.publishCommandDurationObject.publish_manual_override(True)
        if durationMS < 0:
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
        Sends a scaled or raw PWM command to all thrusters.

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
        self.publishCommandDurationObject.publish_pwm_cmd(pwm_set, False, -1)

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

    def timed_pwm(self, time_s: int, pwm_set: List[int], scale: float = 1.0) -> None:
        """
        Sends a PWM command for a fixed duration.

        Args:
            time_s (int): Duration in seconds for which to apply the PWM.
            pwm_set (List[int]): PWM values to apply.
            scale (float): Optional scale factor for PWM values.
        """
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
        print("Executing timed_pwm...")
        self.publishCommandDurationObject.publish_pwm_cmd(pwm_set, True, time_s)

    def read(self) -> None:
        """
        Reads and prints the contents of the PWM command log file.
        """
        with open(self.logFile, "r") as logf:
            file_contents = logf.read()
            print(file_contents)

    def reaction(self, pwm_set: List[int], scale: float = 1.0) -> None:
        """
        Sends scaled PWM values to the plant model for reaction force estimation.

        Args:
            pwm_set (List[int]): PWM command values.
            scale (float): Optional scale factor to modify PWM before processing.
        """
        pwm = [scale * (i - stop_pulse) + stop_pulse for i in pwm_set]
        self.plant.pwm_force(pwm)

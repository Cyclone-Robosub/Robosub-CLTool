import rclpy
from Plant import Plant
from Pwm_Cltool import Pwm_Cltool

import threading
from time import sleep
from typing import List

# Define the CLTool Globals
rev_pulse: int = 1100
stop_pulse: int = 1500
fwd_pulse_raw: int = 1900
rev_adj: float = 1.0
fwd_pulse: int = int(fwd_pulse_raw * rev_adj)
frequency: int = 10
pwm_file: str = "pwm_file.csv"

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

class Thrust_Control:
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

    def __init__(self):
        """
        Initializes ROS, thruster pins, and sets up publishers in a background thread.
        Automatically enables manual mode and provides CLI instructions.
        """
        self.plant = Plant()
        self.thrusters: List[int] = [3, 2, 5, 4, 18, 20, 19, 21]
        rclpy.init()
        self.publishCommandDurationObject = Pwm_Cltool()
        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.start()
        print("Switching onto manual control...")
        sleep(4)
        self.publishCommandDurationObject.publish_manual_switch(True)
        print("Ready to input manual commands")
        print("Please type tcs.exitCLTool() to safely exit manual control.\n")

    def override(self, durationMS: int = -1, pwm_set: List[int] = stop_set):
        """
        Publishes a manual override command, typically for emergency situations.

        Args:
            durationMS (int): Duration in milliseconds. Use -1 for continuous command.
            pwm_set (List[int]): PWM values to apply to thrusters during override.
        """
        self.publishCommandDurationObject.publish_manual_override(True)
        sleep(0.2)
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(durationMS)

    def exitCLTool(self) -> None:
        """
        Safely exits manual control by disabling the manual switch and shutting down ROS.
        """
        print("Shutting down CL Tool and Manual Control")
        self.publishCommandDurationObject.publish_manual_switch(False)
        rclpy.shutdown()

    def spin_ros(self) -> None:
        """
        Spins the ROS node in a separate thread to process incoming/outgoing messages.
        """
        rclpy.spin(self.publishCommandDurationObject)

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
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(-1)

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
        print("Executing timed_pwm function...")
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(time_s)

    def read(self) -> None:
        """
        Reads and prints the contents of the PWM command log file.
        """
        with open(pwm_file, "r") as logf:
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

def main() -> None:
    print("Type in : tcs = Thrust_Control()")

main()

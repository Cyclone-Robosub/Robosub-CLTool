import rclpy
from Plant import Plant
from pwm_cltool import Pwm_Cltool

import threading
import time
from time import sleep

# Define the CLTool Globals
rev_pulse = 1100 * 1
stop_pulse = 1500 * 1
fwd_pulse_raw = (
    1900 * 1
)  # dont use this one, it's output can't be replicated in reverse
rev_adj = 1  # thrusters are more powerful in fwd direction
fwd_pulse = int(fwd_pulse_raw * rev_adj)
frequency = 10
pwm_file = "pwm_file.csv"

zero_set = [0 for i in range(8)]
stop_set = [stop_pulse for i in range(8)]

fwd_set = [stop_pulse for i in range(4)] + [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse]
crab_set = [stop_pulse for i in range(4)] + [fwd_pulse, fwd_pulse, rev_pulse, rev_pulse]
down_set = [fwd_pulse, rev_pulse, fwd_pulse, rev_pulse] + [stop_pulse for i in range(4)]
test_set = [1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536]
barrel = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [stop_pulse for i in range(4)]
summer = [rev_pulse, fwd_pulse, fwd_pulse, rev_pulse] + [stop_pulse for i in range(4)]
spin_set = [stop_pulse for i in range(4)] + [fwd_pulse for i in range(4)]

torpedo = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [
    fwd_pulse,
    rev_pulse,
    fwd_pulse,
    rev_pulse,
]


class Thrust_Control:
    def __init__(self):
        self.plant = Plant()
        # Define PWM pins for each thruster
        self.thrusters = [3, 2, 5, 4, 18, 20, 19, 21]
        rclpy.init()
        self.arraysomething = [3, 2, 5, 4, 18, 20, 19, 21]
        self.publishCommandDurationObject = Pwm_Cltool()
        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.start()
        print("Switching onto manual control...")
        sleep(4)
        self.publishCommandDurationObject.publish_manual_switch(True)
        print("Ready to input manual commands")
        print("Please type tcs.exitCLTool() to safely exit manual control.\n")

    # self.testSendArray(self.publishCommandObject)

    # Set default frequency and duty cycle

    #   for thruster in self.thrusters:
    # thruster.freq(frequency)
    # thruster.duty_ns(0)
    # def testSendArray(self, publishCommandObject):
    # while True:
    # publishCommandObject.publish_array(self.thrusters)

    def override(self, durationMS=-1, pwm_set=stop_set):
        self.publishCommandDurationObject.publish_manual_override(True)
        sleep(0.2)
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(durationMS)

    # TODO: Write auto control and exception handling.
    def exitCLTool(self):
        print("Shutting down CL Tool and Manual Control")
        self.publishCommandDurationObject.publish_manual_switch(False)
        rclpy.shutdown()

    def spin_ros(self):
        rclpy.spin(self.publishCommandDurationObject)

    def pwm(self, pwm_set, scale=1):
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
        if len(pwm_set) != len(self.thrusters):
            print("Wrong length for pwm set\n")
            return
        print("pwm function executed.")
        pwm_set = [int(i) for i in pwm_set]

        # f = open(pwm_file, 'a')
        # start = str(time.time_ns())
        # for i in range(len(pwm_set)):
        #   self.thrusters[i].duty_ns(pwm_set[i])
        # end = str(time.time_ns())
        start = str(time.time_ns())
        # Assuming you have a PWM control library, you would set the duty cycle here
        # For example: self.thrusters[i].duty_ns(pwm_set[i])
        end = str(time.time_ns())
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(-1)

    #  string = start + "," + end + "," + ",".join(map(str, pwm_set)) + "\n"
    #  f.write(string)
    #  print(string)
    #  f.close()

    def scaled_pwm(self, pwm_set, scale):
        new_pwm = [int(scale * (i - stop_pulse) + stop_pulse) for i in pwm_set]

        # if scale < 0:
        #    signs = [-1 * new_pwm[i] / abs(new_pwm[i]) for i in range(len(new_pwm))]
        #    new_pwm = [(new_pwm[i] - stop_pulse) * rev_adj**signs[i] + stop_pulse for i in range(len(new_pwm))]

        return new_pwm

    def timed_pwm(self, time_s, pwm_set, scale=1):
        if scale != 1:
            pwm_set = self.scaled_pwm(pwm_set, scale)
        # with ROS2 send pwm_set array values to a publishing topic.
        print("Executing timed_pwm function...")
        # self.pwm(stop_set)
        self.publishCommandDurationObject.publish_array(pwm_set)
        self.publishCommandDurationObject.publish_duration(time_s)

    # sleep(time_s - (time_s / 2))
    # needs to stop at some point
    # self.publishCommandDurationObject.publish_array(stop_set)
    # self.publishCommandDurationObject.publish_duration(-1)

    def read(self):
        logf = open(pwm_file, "r")
        file_contents = logf.read()
        print(file_contents)
        logf.close()

    def reaction(self, pwm_set, scale=1):
        pwm = [scale * (i - stop_pulse) + stop_pulse for i in pwm_set]
        self.plant.pwm_force(pwm)
        
def main():
    print("Type in : tcs = Thrust_Control()")

main()
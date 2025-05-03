import time
from time import sleep
import math
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
import threading

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
barrell = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [stop_pulse for i in range(4)]
summer = [rev_pulse, fwd_pulse, fwd_pulse, rev_pulse] + [stop_pulse for i in range(4)]
spin_set = [stop_pulse for i in range(4)] + [fwd_pulse for i in range(4)]

torpedo = [fwd_pulse, fwd_pulse, fwd_pulse, fwd_pulse] + [
    fwd_pulse,
    rev_pulse,
    fwd_pulse,
    rev_pulse,
]


class Pwm_Cltool(Node):
    def __init__(self):
        super().__init__("python_cltool_node")
        self.commandPublisher = self.create_publisher(
            Int32MultiArray, "array_Cltool_topic", 10
        )
        self.durationPublisher = self.create_publisher(
            Int64, "duration_Cltool_topic", 10
        )
        self.ManualToggleSwitch = self.create_publisher(Bool, "manual_toggle_switch", 3)
        self.ManualOverride = self.create_publisher(Bool, "manualOverride", 4)

    def publish_array(self, pwm_array):
        msg = Int32MultiArray()
        msg.data = pwm_array
        self.commandPublisher.publish(msg)
        print(msg.data)

    def publish_duration(self, durationSec):
        msg = Int64()
        msg.data = durationSec
        self.durationPublisher.publish(msg)
        print(msg.data)

    def publish_manual_switch(self, isManualEnabled):
        msg = Bool()
        msg.data = isManualEnabled
        self.ManualToggleSwitch.publish(msg)

    #  print(msg.data)
    def publish_manual_override(self, isMistakeMade):
        msg = Bool()
        msg.data = isMistakeMade
        self.ManualOverride.publish(msg)
        print(msg.data)


class Plant:
    def __init__(self):
        # Thruster positions
        self.thruster_positions = [
            [0.2535, -0.2035, 0.042],
            [0.2535, 0.2035, 0.042],
            [-0.2545, -0.2035, 0.042],
            [-0.2545, 0.2035, 0.042],
            [0.1670, -0.1375, -0.049],
            [0.1670, 0.1375, -0.049],
            [-0.1975, -0.1165, -0.049],
            [-0.1975, 0.1165, -0.049],
        ]

        # Thruster directions
        sin45 = math.sin(math.pi / 4)
        self.thruster_directions = [
            [0, 0, 1],
            [0, 0, -1],
            [0, 0, 1],
            [0, 0, -1],
            [-sin45, -sin45, 0],
            [sin45, -sin45, 0],
            [-sin45, sin45, 0],
            [sin45, sin45, 0],
        ]

        # Thruster torques
        self.thruster_torques = [
            self.cross_product(self.thruster_positions[i], self.thruster_directions[i])
            for i in range(8)
        ]

        # Compute wrench matrix (6x8)
        self.wrench_matrix_transposed = [[0] * 6 for _ in range(8)]
        for i in range(8):
            self.wrench_matrix_transposed[i][0:3] = self.thruster_directions[i]
            self.wrench_matrix_transposed[i][3:6] = self.thruster_torques[i]

        # Transpose to get wrench matrix (6x8)
        self.wrench_matrix = self.transpose_matrix(self.wrench_matrix_transposed)

    def pwm_force_scalar(self, x):
        x = x / 1000
        if 1100 <= x < 1460:
            force = (
                (-1.24422882971549e-8) * x**3
                + (4.02057100632393e-5) * x**2
                - 0.0348619861030835 * x
                + 3.90671429105423
            )
        elif 1460 <= x <= 1540:
            force = 0
        elif 1540 < x <= 1900:
            force = (
                (-1.64293565374284e-8) * x**3
                + (9.45962838560648e-5) * x**2
                - 0.170812079190679 * x
                + 98.7232373648272
            )
        else:
            raise ValueError("PWM value out of valid range (1100-1900)")
        return force

    def pwm_force(self, pwm_set):
        thruster_forces = [
            self.pwm_force_scalar(pwm_set[i]) for i in range(len(pwm_set))
        ]
        force = self.matrix_vector_multiply(self.wrench_matrix, thruster_forces)
        print(force)

    @staticmethod
    def transpose_matrix(matrix):
        """Transposes a 2D list (matrix)."""
        return [[row[i] for row in matrix] for i in range(len(matrix[0]))]

    @staticmethod
    def matrix_vector_multiply(matrix, vector):
        """Multiplies a matrix (list of lists) by a vector (list)."""
        return [
            sum(matrix[i][j] * vector[j] for j in range(len(vector)))
            for i in range(len(matrix))
        ]

    @staticmethod
    def cross_product(a, b):
        """Computes the cross product of two 3D vectors a and b."""
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]


class Thrust_Control:
    def __init__(self):
        self.plant = Plant()
        # Define PWM pins for each thruster
        self.thrusters = [3, 2, 5, 4, 18, 20, 19, 21]
        rclpy.init()
        self.arraysomething = [3, 2, 5, 4, 18, 20, 19, 21]
        self.publishCommandDurationObject = PublisherPython()
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

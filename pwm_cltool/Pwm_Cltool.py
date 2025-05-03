from std_msgs.msg import Int32MultiArray
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int64

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

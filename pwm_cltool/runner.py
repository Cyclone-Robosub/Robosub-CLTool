# pwm_cltool/runner.py
import rclpy
from pwm_cltool.pwm_cltool import Pwm_Cltool


def initialize_cltool() -> Pwm_Cltool:
    """
    Initialize rclpy, create the CL Tool instance, and enable manual control.
    """
    clt = Pwm_Cltool()
    return clt


def spin_and_shutdown(clt: Pwm_Cltool) -> None:
    """
    Spin the ROS node and ensure clean shutdown after console exit.
    """
    try:
        # Spin the node to process ROS messages
        clt.start_console()
    finally:
        # When spinning ends (e.g. user exit), shut down cleanly
        clt.exitCLTool()

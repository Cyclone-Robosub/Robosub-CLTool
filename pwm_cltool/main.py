# pwm_cltool/main.py

from pwm_cltool.runner import initialize_cltool, spin_and_shutdown

def main() -> None:
    """
    Entry point for the CLI tool.
    """
    # init the ROS and the CLtool object
    clt = initialize_cltool()
    # handle the node over its lifetime
    spin_and_shutdown(clt)

if __name__ == '__main__':
    main()
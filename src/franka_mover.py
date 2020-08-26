#!/usr/bin/env python2
from franka_control_ros import FrankaRos
import time


def example_position():
    """Used to test if position reporting is working from Arm.

    It will repeatedly print the full arm position data and the XYZ position of the end-effector.
    To use this test, add the ``-p`` or ``--position-example`` flag to the command line.
    """
    arm = FrankaRos(debug=True)
    while True:
        try:
            data = arm.get_position()
            print("End effector position:")
            print("X: ", data[0])
            print("Y: ", data[1])
            print("Z: ", data[2])
            time.sleep(0.4)


if __name__ == '__main__':
    try:
        # example_position()
        
    except KeyboardInterrupt:
        print("\nExiting...")

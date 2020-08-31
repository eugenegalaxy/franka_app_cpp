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
        data = arm.get_position()
        print("End effector position:")
        print("X: ", data[0])
        print("Y: ", data[1])
        print("Z: ", data[2])
        time.sleep(0.4)


def example_move_relative(dx, dy, dz, speed):
    arm = FrankaRos(debug=True)
    arm.move_relative(dx, dy, dz, speed)


def move_gripper(width, speed):
    arm = FrankaRos(debug=True)
    arm.move_gripper(width, speed)


if __name__ == '__main__':
    try:
        # example_position()
        # move_gripper(0.05, 0.1)
        example_move_relative(0.05, 0, 0, 0.1)
    except KeyboardInterrupt:
        print("\nExiting...")

#!/usr/bin/env python2
'''
This is a client server for Franka Gripper Move command.
Before running this script, don't forget to roslaunch Gripper node (either via franka_control or gripper):
roslaunch franka_control franka_control.launch load_gripper:=true robot_ip:=172.16.0.2
or only gripper:
roslaunch franka_gripper franka_gripper.launch robot_ip:=172.16.0.2

How to use:
rosrun franka_app_cpp fr_grip_move_action.py --width 0.1 --speed 0.15

This script was created by Jevgenijs Galaktionovs, AAU employee.
Email: jgalak16@student.aau.dk (or) accidentalyo@gmail.com
GitHub repo: https://github.com/eugenegalaxy/franka_app_cpp

Date: 26.08.2020
'''

import rospy
import actionlib
import argparse

from franka_gripper.msg import MoveGoal, MoveAction


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=float, help='Goal gripper width in meters')
    parser.add_argument('--speed', type=float, default=0.05, help='Goal gripper speed in meters per second')
    args = parser.parse_args()

    rospy.init_node('Franka_gripper_move_action')
    client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    client.wait_for_server()
    rospy.loginfo('Moving the gripper to {0}m width with {1}m/s speed.'.format(args.width, args.speed))
    goal = MoveGoal(args.width, args.speed)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

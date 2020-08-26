#!/usr/bin/env python2
'''
This is a client server for Franka Gripper GRASP command.
Before running this script, don't forget to roslaunch Gripper node (either via franka_control or gripper):
roslaunch franka_control franka_control.launch load_gripper:=true robot_ip:=172.16.0.2
or only gripper:
roslaunch franka_gripper franka_gripper.launch robot_ip:=172.16.0.2

How to use:
rosrun franka_app_cpp fr_grip_grasp_action.py --width 0.1 --speed 0.15 --force 30 --epsilon_inner 0.1 --epsilon_outer 0.1
(only --width argument is required. The rest is optional.)

This script was created by Jevgenijs Galaktionovs, AAU employee.
Email: jgalak16@student.aau.dk (or) accidentalyo@gmail.com
GitHub repo: https://github.com/eugenegalaxy/franka_app_cpp

Date: 26.08.2020
'''

import rospy
import actionlib
import argparse

from franka_gripper.msg import GraspGoal, GraspAction, GraspEpsilon


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=float, help='Goal gripper width in meters')
    parser.add_argument('--speed', type=float, default=0.1, help='Goal gripper speed in meters per second')
    parser.add_argument('--force', type=int, default=20, help='Goal gripper grasping torque in Newtons')
    parser.add_argument('--epsilon_inner', type=float, default=0.005, help='Width error inner allowance in meters \
                                                                    (How much more  than "width" gripper can squeeze)')
    parser.add_argument('--epsilon_outer', type=float, default=0.005, help='Width error outer allowance in meters \
                                                                    (How much less than "width" gripper can squeeze)')

    args = parser.parse_args()
    epsilon = GraspEpsilon()
    epsilon.inner = args.epsilon_inner
    epsilon.outer = args.epsilon_outer

    rospy.init_node('Franka_gripper_move_action')
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()
    rospy.loginfo('Grasping object at {0}m width with {1}m/s speed and {2}N force.\
        '.format(args.width, args.speed, args.force))
    rospy.loginfo('Epsilon inner={0}, Epsilon outer={1}'.format(epsilon.inner, epsilon.outer))
    goal = GraspGoal(args.width, epsilon, args.speed, args.force)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

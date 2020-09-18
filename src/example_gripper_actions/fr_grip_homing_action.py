#!/usr/bin/env python2
'''
This is a client server for Franka Gripper HOMING command.
Before running this script, don't forget to roslaunch Gripper node (either via franka_control or gripper):
roslaunch franka_control franka_control.launch load_gripper:=true robot_ip:=172.16.0.2
or only gripper:
roslaunch franka_gripper franka_gripper.launch robot_ip:=172.16.0.2

How to use:
rosrun franka_app_cpp fr_grip_homing_action.py

This script was created by Jevgenijs Galaktionovs, AAU employee.
Email: jgalak16@student.aau.dk (or) accidentalyo@gmail.com
GitHub repo: https://github.com/eugenegalaxy/franka_app_cpp

Date: 26.08.2020
'''

import rospy
import actionlib
from franka_gripper.msg import HomingAction, HomingGoal


if __name__ == '__main__':
    rospy.init_node('Franka_gripper_homing_action')
    client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
    client.wait_for_server()
    rospy.loginfo('Homing the gripper.')
    goal = HomingGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

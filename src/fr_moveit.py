import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_group_python_interface', anonymous=True)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# group_name = "panda_arm"
# group = moveit_commander.MoveGroupCommander(group_name)
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#     moveit_msgs.msg.DisplayTrajectory, queue_size=20)


# https://usermanual.wiki/Document/PandaProgrammingGuide.1809781175/help

# NOTHING IS FINISHED HERE

from franka.franka_control_ros import FrankaRos

franka = FrankaRos(debug_flag=True)
# we set the flag true to get prints to the console about what FrankaRos is doing

while True:
   data = arm.get_position()
   print("End effector position:")
   print("X: ", data[0])
   print("Y: ", data[1])
   print("Z: ", data[2])
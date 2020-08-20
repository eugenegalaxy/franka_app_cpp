#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/HomingAction.h>



int main(int argc, char** argv)
{
//   ros::init(argc, argv, "franka_gripper_client");
//   // create the action client
//   // true causes the client to spin its own thread
//   actionlib::SimpleActionClient<franka_gripper::HomingAction> client("franka_gripper", true);

//   ROS_INFO("Waiting for action server to start.");
//   // wait for the action server to start
//   client.waitForServer(); //will wait for infinite time

//   ROS_INFO("Action server started, sending goal.");
//   // send a goal to the action

// //   franka_gripper::HomingGoal goal;
//   franka_gripper::HomingActionGoal goal;
// //   goal.
// //   goal.speed = 0.1;

//   client.sendGoal(goal);
//   client.waitForResult(ros::Duration(5.0));
//   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     printf("Yay! The gripper is moved");
//   printf("Current State: %s\n", client.getState().toString().c_str());
//   return 0;
}
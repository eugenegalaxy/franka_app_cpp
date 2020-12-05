// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <limits> // for quiet_NaN

#include <franka/exception.h>
#include <franka/robot.h>

#include <franka_app/examples_common.h>

#define ROBOT_IP_STR "172.27.23.65"

double target_x = std::numeric_limits<double>::quiet_NaN(); 
double target_y = std::numeric_limits<double>::quiet_NaN(); 
double target_z = std::numeric_limits<double>::quiet_NaN(); 
double speed = 0.0;  

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  // if (argc != 6) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z> <velocity>" << std::endl;
  //   return -1;
  // }
  try {
    franka::Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // // Set additional parameters always before the control loop, NEVER in the control loop!
    // // Set collision behavior.
    // robot.setCollisionBehavior(
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //     {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // std::array<double, 16> initial_pose;
    // double time = 0.0;

    // std::string arg2 = argv[2];
    // std::string arg3 = argv[3];
    // std::string arg4 = argv[4];
    // std::string arg5 = argv[5];
    // target_x = std::stod(arg2);
    // target_y = std::stod(arg3);
    // target_z = std::stod(arg4);
    // speed = std::stod(arg5);
    

    // // TODO: This stuff doesnt work, but keep fixing it until it does...

    // robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
    //                                      franka::Duration period) -> franka::CartesianPose {
    //   time += period.toSec();

    //   if (time == 0.0) {
    //     initial_pose = robot_state.O_T_EE_c;
    //   }

    //   std::array<double, 16> current_pose = initial_pose;
    //   double cur_x = current_pose[12]; 
    //   double cur_y = current_pose[13];
    //   double cur_z = current_pose[14]; 

    //       // computing the motion
    //   double vec_x = target_x - cur_x;
    //   double vec_y = target_y - cur_y;
    //   double vec_z = target_z - cur_z; 

    //   double l2_norm = sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);

    //   double new_x = 0;
    //   double new_y = 0;
    //   double new_z = 0;

    //   new_x = (vec_x / l2_norm);
    //   new_y = (vec_y / l2_norm);
    //   new_z = (vec_z / l2_norm); 

    //   // initially, the robot moves to its current position (-> no motion)
    //   if (std::isnan(target_x)) {
    //     target_x = cur_x; 
    //     target_y = cur_y; 
    //     target_z = cur_z; 
    //   }

    //   std::array<double, 16> new_pose = initial_pose;
    //   new_pose[12] += new_x;
    //   new_pose[13] += new_y;
    //   new_pose[14] += new_z;

    //   // constexpr double kRadius = 0.3;
    //   // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
    //   // double delta_x = kRadius * std::sin(angle);
    //   // double delta_z = kRadius * (std::cos(angle) - 1);

    //   // std::array<double, 16> new_pose = initial_pose;
    //   // new_pose[12] += delta_x;
    //   // new_pose[14] += delta_z;

    //   if (time >= 3.0) {
    //     std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    //     return franka::MotionFinished(new_pose);
    //   }
    //   return new_pose;
    // });
    } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
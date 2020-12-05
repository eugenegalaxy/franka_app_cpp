// // Copyright (c) 2017 Franka Emika GmbH
// // Use of this source code is governed by the Apache-2.0 license, see LICENSE
// #include <cmath>
// #include <iostream>

// #include <limits> // for quiet_NaN

// #include <franka/exception.h>
// #include <franka/robot.h>

// #include <franka_app/examples_common.h>

// #define ROBOT_IP_STR "172.27.23.65"

// double target_x = std::numeric_limits<double>::quiet_NaN(); 
// double target_y = std::numeric_limits<double>::quiet_NaN(); 
// double target_z = std::numeric_limits<double>::quiet_NaN(); 
// double speed = 0.0;  

// /**
//  * @example generate_joint_motion.cpp
//  * An example showing how to generate a Joint Pose motion.
//  *
//  * @warning Before executing this example, make sure there is enough space in front of the robot.
//  */

// int main(int argc, char** argv) {
//   // if (argc != 6) {
//   //   std::cerr << "Usage: " << argv[0] << " <robot-hostname> <x> <y> <z> <velocity>" << std::endl;
//   //   return -1;
//   // }
//   try {
//     franka::Robot robot(argv[1]);
//     robot.automaticErrorRecovery();
//     setDefaultBehavior(robot);

//     // First move the robot to a suitable joint configuration
//     std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
//     MotionGenerator motion_generator(0.5, q_goal);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(motion_generator);
//     std::cout << "Finished moving to initial joint configuration." << std::endl;


//     } catch (const franka::Exception& e) {
//     std::cout << e.what() << std::endl;
//     return -1;
//   }

//   return 0;
// }
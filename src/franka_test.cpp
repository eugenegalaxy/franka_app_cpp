#include <franka/robot.h>
#include <franka/gripper.h>
#include <iostream>

#define ROBOT_IP_STR "172.16.0.2"



int main() {
    franka::Gripper gripper(ROBOT_IP_STR); // Connect to gripper
    franka::Robot robot(ROBOT_IP_STR); // Connect to robot

    ////========================================================
    //// GRIPPER manipulations.

    // gripper.homing(); // Gripper Calibration

    // double move_width1 = 0.1; // in meters
    // double move_speed1 = 0.20; // m/s
    // gripper.move(move_width1, move_speed1);

    // double move_width2 = 0.02; // meters
    // double move_speed2 = 0.05; // meters/sec
    // double grasp_force = 10; // Newtons
    // bool result = gripper.grasp(move_width2, move_speed2, grasp_force);
    // std::cout << result << std::endl; // '0' -> Target not reached. '1' -> Success

    // gripper.stop(); // Simply stops gripper

    franka::GripperState gripper_state = gripper.readOnce();
    std::cout << gripper_state << std::endl;
    
    ////========================================================

    // robot.automaticErrorRecovery(); // Works only in BLUE light mode ("enabled for motion")
    

    ////========================================================
    // std::cout<<"Zero torque mode.\
    //             Gravity is compensated by the robot itself! "<<std::endl;
    // robot.control([&](const franka::RobotState&, franka::Duration) -> franka::Torques {
    // return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // });


    ////========================================================
    // A single sample of the robot state
    // franka::RobotState state = robot.readOnce();
    // std::cout << state << std::endl;


    ////========================================================
    // // This loop will continuously read the robot state.
    // // Code snippet from "echo_robot_state" example
    // size_t count = 0;
    // robot.read([&count](const franka::RobotState& robot_state) {
    // // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
    // // but should not be done in a control loop.
    // std::cout << robot_state << std::endl;
    // return count++ < 100;
    // });


    ////========================================================
    

    std::cout << "Press enter key to close the program.\n";
    getchar();
    std::cout << "Closing connection with Robot and Gripper. Vi ses!\n";
    return 0;
}
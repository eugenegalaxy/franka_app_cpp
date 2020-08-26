/*
This is an example library for Franka robot/gripper control in C++

How to use:
1. Uncomment the section you want to test (TODO: functions)
2. catkin_make franka_app
2. rosrun franka_app_cpp fr_grip_homing_action.py

This script was created by Jevgenijs Galaktionovs, AAU employee.
Email: jgalak16@student.aau.dk (or) accidentalyo@gmail.com
GitHub repo: https://github.com/eugenegalaxy/franka_app_cpp

Date: 26.08.2020
*/


#include <iostream>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>


#define ROBOT_IP_STR "172.16.0.2"

franka::Torques
zero_torque_callback( const franka::RobotState&, franka::Duration)
{
    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}


void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

int main() {
    
try{
    franka::Gripper gripper(ROBOT_IP_STR); // Connect to gripper
    franka::Robot robot(ROBOT_IP_STR); // Connect to robot
    robot.automaticErrorRecovery(); // Works only in BLUE light mode ("enabled for motion")<    
    setDefaultBehavior(robot);
    // Recovers robot from failed states. E.g. -> When it blocks itself due to collision.


    double time = 0.0;

    std::array<double, 16> initial_pose;

    robot.control( [&initial_pose, &time](const franka::RobotState& robot_state,
        franka::Duration time_step) -> franka::CartesianPose{

        time += time_step.toSec();

        if (time == 0.0) {
            initial_pose = robot_state.O_T_EE_c;
        }


        constexpr double kRadius = 0.3;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
        double delta_x = kRadius * std::sin(angle);
        double delta_z = kRadius * (std::cos(angle) - 1);
        std::array<double, 16> new_pose = initial_pose;
        new_pose[12] += delta_x;
        std::cout << new_pose[12] << std::endl;
        new_pose[14] += delta_z;


        if(time >= 10){
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
        }
        return new_pose;
    });
    
    ////========================================================
    //// Poorly working Joint Position example
    // robot.control( [=, &time](const franka::RobotState& robot_state,
    //     franka::Duration time_step) -> franka::JointPositions
    // {
    //     time += time_step.toSec();  // Update time at the beginning of the callback.
    //     franka::JointPositions output = {{0,0,0,0,0,3.14,0}}; // radians

    //     int MAX_TIME = 5; // sec
    //     if (time >= MAX_TIME) { 
    //         // Return MotionFinished at the end of the trajectory.
    //         return franka::MotionFinished(output);
    //     }
    //     return output;
    // }, franka::ControllerMode::kJointImpedance);

    ////========================================================
    // Velocity joint motion example. Does the move with last 4 joints and goes back.
    // Control amplitude and time with omega_max and time_max.
    // robot.control(
    //  [=, &time](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
    //    time += period.toSec();
    //    int time_max = 2;
    //    double omega_max = 1;
    //    double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
    //    double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

    //    franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};

    //    if (time >= 2 * time_max) {
    //      std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    //      return franka::MotionFinished(velocities);
    //    }
    //    return velocities;
    //  });

    ////========================================================
    // std::cout<<"Zero torque mode.\
    // robot.control(zero_torque_callback);

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

    // franka::GripperState gripper_state = gripper.readOnce();
    // std::cout << gripper_state << std::endl;
    
    ////========================================================

} catch (franka::Exception const& e) 
    {
    std::cout << e.what() << std::endl;
    return -1;
    }
    std::cout << "Press enter key to close the program.\n";
    getchar();
    std::cout << "Closing connection with Robot and Gripper. Vi ses!\n";
    return 0;
}
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <opencv2/opencv.hpp>

#define ROBOT_IP_STR "172.16.0.2"

// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
    x = 0;
    y = CV_PI/2;
    z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
    x = 0;
    y = -CV_PI/2;
    z = atan2(m02,m22);
  }
  else
  {
    x = atan2(-m12,m11);
    y = asin(m10);
    z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "franka_position_publisher");
    ros::NodeHandle node_handle;
    ros::Publisher pub_positions = node_handle.advertise<std_msgs::Float64MultiArray>("franka_current_position", 1);
    ros::Rate loop_rate(100);  // 2 kHz

        while (ros::ok())
        {
            std_msgs::Float64MultiArray msg;

            
            
            franka::Robot robot(ROBOT_IP_STR);
            std::array<double, 16> pose;
            cv::Mat eulers(3, 1, CV_64F);
            size_t count = 0;
            // cv::Mat_<double> rotationMatrix(3,3, 0.0);

            robot.read([&count, &pose, &eulers](const franka::RobotState& robot_state) {
                auto state_pose = robot_state.O_T_EE_d;
                pose = state_pose;

            std::array<double, 9> rotation_vector;
            rotation_vector = {pose[0],pose[1], pose[2],
                               pose[4],pose[5], pose[6],
                               pose[8],pose[9], pose[10]};

            cv::Mat rotationMatrix(3, 3, CV_64FC1);
            for(int i = 0; i < 3; i++) 
                for(int j = 0; j < 3; j++) 
                    rotationMatrix.at<double>(i,j)= rotation_vector[i+j];
            eulers = rot2euler(rotationMatrix);

                return 0; 
            });

            msg.data = {pose[12],pose[13],pose[14], eulers.at<double>(0), eulers.at<double>(1), eulers.at<double>(2)}; // EE_xyz in meters
            pub_positions.publish(msg);

            ros::spinOnce();
            loop_rate.sleep(); 
        }
    return 0;
}
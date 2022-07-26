#include "dynamixel_controller/dynamixel_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_controller_node");
    ros::MultiThreadedSpinner spinner(2);
    DynamixelController dynamixel_controller;   
    spinner.spin();

    ROS_INFO("yo");

    return 0;
}
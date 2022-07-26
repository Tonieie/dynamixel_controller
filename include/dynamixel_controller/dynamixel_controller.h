#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define ARM_TORQUE_ENABLE_ADDR		512
#define GRIPPER_TORQUE_ENABLE_ADDR	64

#define GRIPPER_GOAL_POSITION_ADDR	116
#define ARM_GOAL_POSITION_ADDR		564

#define GRIPPER_PRESENT_POSITION_ADDR   132
#define ARM_PRESENT_POSITION_ADDR		580

#define MAX_ARM_ID 3
#define MAX_GRIPPER_ID 6

class DynamixelController
{
	public:
		DynamixelController();   

	private:
		boost::shared_ptr<ros::NodeHandle> nh;
		boost::shared_ptr<ros::NodeHandle> _nh;
		std::string device_name;
		int baudrate;

		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;
		boost::shared_ptr<dynamixel::GroupSyncWrite> armSyncWrite;
		boost::shared_ptr<dynamixel::GroupSyncWrite> gripperSyncWrite;
		boost::shared_ptr<dynamixel::GroupSyncRead> armSyncRead;
		boost::shared_ptr<dynamixel::GroupSyncRead> gripperSyncRead;

		double cmd[7];
		double pos[7];
		double vel[7];
		double eff[7];

		union Int32ToByte
		{
			int32_t asInt;
			uint8_t asByte[4];
		}cmd_to_send;

		void initDynamixel();
		void writeArm();
		void writeGripper();
		void readArm();
		void readGripper();
};

#endif
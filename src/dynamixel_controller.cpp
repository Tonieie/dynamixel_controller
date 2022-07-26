#include "dynamixel_controller/dynamixel_controller.h"

DynamixelController::DynamixelController()
{
    this->nh = boost::make_shared<ros::NodeHandle>();

    this->_nh = boost::make_shared<ros::NodeHandle>("~");
    _nh->param<std::string>("device_name", device_name, "/dev/ttyUSB0");
    _nh->param<int>("baudrate", baudrate, 115200);

    initDynamixel();
}

void DynamixelController::initDynamixel()
{
    this->portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    this->armSyncRead = boost::make_shared<dynamixel::GroupSyncRead>(portHandler, packetHandler, ARM_PRESENT_POSITION_ADDR, 4);
    this->gripperSyncRead = boost::make_shared<dynamixel::GroupSyncRead>(portHandler, packetHandler, GRIPPER_PRESENT_POSITION_ADDR, 4);
    this->armSyncWrite = boost::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler, ARM_GOAL_POSITION_ADDR, 4);
    this->gripperSyncWrite = boost::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler, GRIPPER_GOAL_POSITION_ADDR, 4);

    if (portHandler->openPort())
    {
        ROS_INFO("Succeeded to open the port!");
    }
    else
    {
        ROS_WARN("Failed to open the port!");
    }

    if (portHandler->setBaudRate(baudrate))
    {
        ROS_INFO("Succeeded to change the baudrate to %d!", baudrate);
    }
    else
    {
        ROS_WARN("Failed to change the baudrate!");
    }

    uint8_t dxl_error = 0;                          // Dynamixel error
    int dxl_comm_result = 0;
    for(size_t id = 1; id < 7; id++)
    {
        if(id <= MAX_ARM_ID)
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ARM_TORQUE_ENABLE_ADDR, 1, &dxl_error);
        else if(id <= MAX_GRIPPER_ID)
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, GRIPPER_TORQUE_ENABLE_ADDR, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            ROS_INFO("%s", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            ROS_INFO("%s", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            ROS_INFO("Dynamixel#%d has been successfully connected", id);
        }
    }
}
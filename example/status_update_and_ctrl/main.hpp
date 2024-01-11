#ifndef STATUS_UPDATE_AND_CTRL_HPP
#define STATUS_UPDATE_AND_CTRL_HPP
#include <ros/ros.h>
#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_STATUS.h"

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "diablo_utils/diablo_tools/onboard_sdk_uart_protocol.h"

#include <std_msgs/String.h>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

namespace info_update_and_ctrl
{
    class InfoUpdateAndCtrl
    {
    public:
        int SubscribeAndPublish();

        void teleopCtrlCallBack(const std_msgs::String::ConstPtr &msg);

        void publishDataProcess(DIABLO::OSDK::Vehicle *vehicle_);

    private:
        ros::NodeHandle n_;
        ros::Subscriber sub_;
        ros::Publisher ACCLPublisher;
        ros::Publisher GYROPublisher;
        ros::Publisher LEGMOTORSPublisher;
        ros::Publisher POWERPublisher;
        ros::Publisher QUATERNIONPublisher;
        ros::Publisher STATUSPublisher;
        DIABLO::OSDK::Movement_Ctrl *movement_ctrl_;
        DIABLO::OSDK::Vehicle *vehicle;
    }; // End of class InfoUpdateAndCtrl
} // namespace info_update_and_ctrl

#endif // STATUS_UPDATE_AND_CTRL_HPP

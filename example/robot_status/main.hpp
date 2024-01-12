#ifndef ROBOT_STATUS__HPP
#define ROBOT_STATUS__HPP
#include <ros/ros.h>
#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_STATUS.h"

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"
#include "diablo_utils/diablo_tools/onboard_sdk_uart_protocol.h"

namespace info_update
{
    class InfoUpdate
    {
    public:
        int start_robot_status_update();
        void publishDataProcess(DIABLO::OSDK::Vehicle *vehicle_);

    private:
        ros::NodeHandle n_;
        ros::Publisher ACCLPublisher;
        ros::Publisher GYROPublisher;
        ros::Publisher LEGMOTORSPublisher;
        ros::Publisher POWERPublisher;
        ros::Publisher QUATERNIONPublisher;
        ros::Publisher STATUSPublisher;
    }; // End of class InfoUpdateAndCtrl
} // namespace info_update

#endif // ROBOT_STATUS__HPP

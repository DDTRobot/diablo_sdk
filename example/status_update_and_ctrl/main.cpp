#include "main.hpp"

namespace info_update_and_ctrl
{
    int InfoUpdateAndCtrl::SubscribeAndPublish()
    {
        DIABLO::OSDK::HAL_Pi Hal;
        if (Hal.init())
            return -1;

        vehicle = new DIABLO::OSDK::Vehicle(&Hal); // Initialize Onboard SDK
        if (vehicle->init())
            return -1;

        vehicle->telemetry->activate();

        vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
        vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
        vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
        vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
        vehicle->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);

        vehicle->telemetry->configUpdate();

        movement_ctrl_ = vehicle->movement_ctrl;

        // Topic you want to publish
        ACCLPublisher = n_.advertise<diablo_sdk::OSDK_ACCL>("diablo_ros_ACCL_b", 10);
        GYROPublisher = n_.advertise<diablo_sdk::OSDK_GYRO>("diablo_ros_GYRO_b", 10);
        LEGMOTORSPublisher = n_.advertise<diablo_sdk::OSDK_LEGMOTORS>("diablo_ros_LEGMOTORS_b", 10);
        POWERPublisher = n_.advertise<diablo_sdk::OSDK_POWER>("diablo_ros_POWER_b", 10);
        QUATERNIONPublisher = n_.advertise<diablo_sdk::OSDK_QUATERNION>("diablo_ros_QUATERNION_b", 10);
        STATUSPublisher = n_.advertise<diablo_sdk::OSDK_STATUS>("diablo_ros_STATUS_b", 10);

        // Topic you want to subscribe
        sub_ = n_.subscribe("/DJ_teleop", 1, &InfoUpdateAndCtrl::teleopCtrlCallBack, this);

        ros::spin();
    }

    void InfoUpdateAndCtrl::teleopCtrlCallBack(const std_msgs::String::ConstPtr &msg)
    {

        if (!movement_ctrl_->in_control())
        {
            printf("Try to get the control of robot movement!.\n");
            uint8_t result = movement_ctrl_->obtain_control();
            return;
        }
        if (movement_ctrl_->ctrl_mode_data.height_ctrl_mode == 1)
            movement_ctrl_->ctrl_data.up = 0.0f;
        movement_ctrl_->ctrl_data.forward = 0.0f;
        movement_ctrl_->ctrl_data.left = 0.0f;
        for (const char &c : msg->data)
        {
            switch (c)
            {
            case 'w':
                movement_ctrl_->ctrl_data.forward = 1.0f; // vel ctrl
                break;
            case 'a':
                movement_ctrl_->ctrl_data.left = 1.0f; // angular_vel ctrl
                break;
            case 's':
                movement_ctrl_->ctrl_data.forward = -1.0f; // vel ctrl
                break;
            case 'd':
                movement_ctrl_->ctrl_data.left = -1.0f; // angular_vel ctrl
                break;
            case 'q':
                movement_ctrl_->ctrl_data.roll = -0.1f; // pos ctrl
                break;
            case 'e':
                movement_ctrl_->ctrl_data.roll = 0.1f; // pos ctrl
                break;
            case 'r':
                movement_ctrl_->ctrl_data.roll = 0.0f; // pos ctrl
                break;

            case 'h':
                movement_ctrl_->ctrl_data.up = -0.5f;
                break;
            case 'j':
                movement_ctrl_->ctrl_data.up = 1.0f;
                break;
            case 'k':
                movement_ctrl_->ctrl_data.up = 0.5f;
                break;
            case 'l':
                movement_ctrl_->ctrl_data.up = 0.0f;
                break;

            case 'u':
                movement_ctrl_->ctrl_data.pitch = 0.5f;
                break;
            case 'i':
                movement_ctrl_->ctrl_data.pitch = 0.0f;
                break;
            case 'o':
                movement_ctrl_->ctrl_data.pitch = -0.5f;
                break;

            case 'v':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->ctrl_mode_data.height_ctrl_mode = 1;
                break;
            case 'b':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->ctrl_mode_data.height_ctrl_mode = 0;
                break;
            case 'n':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->ctrl_mode_data.pitch_ctrl_mode = 1;
                break;
            case 'm':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->ctrl_mode_data.pitch_ctrl_mode = 0;
                break;

            case 'z':
                movement_ctrl_->SendTransformUpCmd();
                movement_ctrl_->ctrl_data.up = 1.0f;
                break;
            case 'x':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->SendTransformDownCmd();
                break;
            case 'c':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->SendJumpCmd(true);
                sleep(1); // wait for jump charge!
                break;
            case 'f':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->SendDanceCmd(true);
                break;
            case 'g':
                movement_ctrl_->ctrl_mode_cmd = true;
                movement_ctrl_->SendDanceCmd(false);
                break;
            default:
                movement_ctrl_->ctrl_mode_cmd = false;
                movement_ctrl_->SendJumpCmd(false);
            }
        }

        if (movement_ctrl_->ctrl_mode_cmd)
        {
            uint8_t result = movement_ctrl_->SendMovementModeCtrlCmd();
        }
        else
        {
            uint8_t result = movement_ctrl_->SendMovementCtrlCmd();
        }
        InfoUpdateAndCtrl::publishDataProcess(vehicle);
    }

    void InfoUpdateAndCtrl::publishDataProcess(DIABLO::OSDK::Vehicle *vehicle_)
    {
        ros::Rate loop_rate(100);
        while (ros::ok())
        {
            if (vehicle_->telemetry->newcome & 0x40)
            {
                diablo_sdk::OSDK_STATUS msg;
                msg.ctrl_mode = vehicle_->telemetry->status.ctrl_mode;
                msg.robot_mode = vehicle_->telemetry->status.robot_mode;
                msg.error = vehicle_->telemetry->status.error;
                msg.warning = vehicle_->telemetry->status.warning;
                STATUSPublisher.publish(msg);
                vehicle_->telemetry->eraseNewcomeFlag(0xBF);
            }
            if (vehicle_->telemetry->newcome & 0x20)
            {
                diablo_sdk::OSDK_QUATERNION msg;
                msg.w = vehicle_->telemetry->quaternion.w;
                msg.x = vehicle_->telemetry->quaternion.x;
                msg.y = vehicle_->telemetry->quaternion.y;
                msg.z = vehicle_->telemetry->quaternion.z;
                QUATERNIONPublisher.publish(msg);
                printf("Quaternion_w:\t%f\nQuaternion_x:\t%f\nQuaternion_y:\t%f\nQuaternion_z:\t%f\n", msg.w, msg.x, msg.y, msg.z);
                vehicle_->telemetry->eraseNewcomeFlag(0xDF);
            }
            if (vehicle_->telemetry->newcome & 0x10)
            {
                diablo_sdk::OSDK_ACCL msg;
                msg.x = vehicle_->telemetry->accl.x;
                msg.y = vehicle_->telemetry->accl.y;
                msg.z = vehicle_->telemetry->accl.z;
                ACCLPublisher.publish(msg);
                printf("ACCL_X:\t%f\nACCL_Y:\t%f\nACCL_Z:\t%f\n", msg.x, msg.y, msg.z);
                vehicle_->telemetry->eraseNewcomeFlag(0xEF);
            }
            if (vehicle_->telemetry->newcome & 0x08)
            {
                diablo_sdk::OSDK_GYRO msg;
                msg.x = vehicle_->telemetry->gyro.x;
                msg.y = vehicle_->telemetry->gyro.y;
                msg.z = vehicle_->telemetry->gyro.z;
                GYROPublisher.publish(msg);
                printf("GYRO_X:\t%f\nGYRO_Y:\t%f\nGYRO_Z:\t%f\n", msg.x, msg.y, msg.z);
                vehicle_->telemetry->eraseNewcomeFlag(0xF7);
            }
            if (vehicle_->telemetry->newcome & 0x02)
            {
                diablo_sdk::OSDK_POWER msg;
                msg.battery_voltage = vehicle_->telemetry->power.voltage;
                msg.battery_current = vehicle_->telemetry->power.current;
                msg.battery_capacitor_energy = vehicle_->telemetry->power.capacitor_energy;
                msg.battery_power_percent = vehicle_->telemetry->power.power_percent;
                POWERPublisher.publish(msg);
                printf("Power:\nVoltage:\t%f\nCurrent:\t%f\nCap_EN:\t%f\nPercent:\t%u\n", msg.battery_voltage, msg.battery_current, msg.battery_current, msg.battery_power_percent);
                vehicle_->telemetry->eraseNewcomeFlag(0xFD);
            }
            if (vehicle_->telemetry->newcome & 0x01)
            {
                diablo_sdk::OSDK_LEGMOTORS msg;
                msg.left_hip_enc_rev = vehicle_->telemetry->motors.left_hip.rev;
                msg.left_hip_pos = vehicle_->telemetry->motors.left_hip.pos;
                msg.left_hip_vel = vehicle_->telemetry->motors.left_hip.vel;
                msg.left_hip_iq = vehicle_->telemetry->motors.left_hip.iq;

                msg.left_knee_enc_rev = vehicle_->telemetry->motors.left_knee.rev;
                msg.left_knee_pos = vehicle_->telemetry->motors.left_knee.pos;
                msg.left_knee_vel = vehicle_->telemetry->motors.left_knee.vel;
                msg.left_knee_iq = vehicle_->telemetry->motors.left_knee.iq;

                msg.left_wheel_enc_rev = vehicle_->telemetry->motors.left_wheel.rev;
                msg.left_wheel_pos = vehicle_->telemetry->motors.left_wheel.pos;
                msg.left_wheel_vel = vehicle_->telemetry->motors.left_wheel.vel;
                msg.left_wheel_iq = vehicle_->telemetry->motors.left_wheel.iq;

                msg.right_hip_enc_rev = vehicle_->telemetry->motors.right_hip.rev;
                msg.right_hip_pos = vehicle_->telemetry->motors.right_hip.pos;
                msg.right_hip_vel = vehicle_->telemetry->motors.right_hip.vel;
                msg.right_hip_iq = vehicle_->telemetry->motors.right_hip.iq;

                msg.right_knee_enc_rev = vehicle_->telemetry->motors.right_knee.rev;
                msg.right_knee_pos = vehicle_->telemetry->motors.right_knee.pos;
                msg.right_knee_vel = vehicle_->telemetry->motors.right_knee.vel;
                msg.right_knee_iq = vehicle_->telemetry->motors.right_knee.iq;

                msg.right_wheel_enc_rev = vehicle_->telemetry->motors.right_wheel.rev;
                msg.right_wheel_pos = vehicle_->telemetry->motors.right_wheel.pos;
                msg.right_wheel_vel = vehicle_->telemetry->motors.right_wheel.vel;
                msg.right_wheel_iq = vehicle_->telemetry->motors.right_wheel.iq;

                LEGMOTORSPublisher.publish(msg);
                vehicle_->telemetry->eraseNewcomeFlag(0xFE);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
} // namespace info_update_and_ctrl

int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "status_update_and_ctrl_example");

    info_update_and_ctrl::InfoUpdateAndCtrl test;
    test.SubscribeAndPublish();

    return 0;
}




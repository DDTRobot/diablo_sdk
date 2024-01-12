#include "main.hpp"

namespace info_update
{
int InfoUpdate::start_robot_status_update()
{ 
    DIABLO::OSDK::HAL_Pi Hal;
    while(Hal.init()!=0);
    std::cout<<"Hal init.\n"<<std::endl;
    DIABLO::OSDK::Vehicle *vehicle;  
    vehicle = new DIABLO::OSDK::Vehicle(&Hal); // Initialize Onboard SDK
    InfoUpdate::publishDataProcess(vehicle);
}
void InfoUpdate::publishDataProcess(DIABLO::OSDK::Vehicle *vehicle_)
{
    while(vehicle_->init()!=0);
    std::cout<<"vehicle init.\n"<<std::endl;
    vehicle_->telemetry->activate();

    vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
    vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
    vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
    vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
    vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);

    vehicle_->telemetry->configUpdate();
    std::cout<<"configUpdate.\n"<<std::endl;
    // Topic you want to publish
    ACCLPublisher = n_.advertise<diablo_sdk::OSDK_ACCL>("diablo_ros_ACCL_b", 10);
    GYROPublisher = n_.advertise<diablo_sdk::OSDK_GYRO>("diablo_ros_GYRO_b", 10);
    LEGMOTORSPublisher = n_.advertise<diablo_sdk::OSDK_LEGMOTORS>("diablo_ros_LEGMOTORS_b", 10);
    POWERPublisher = n_.advertise<diablo_sdk::OSDK_POWER>("diablo_ros_POWER_b", 10);
    QUATERNIONPublisher = n_.advertise<diablo_sdk::OSDK_QUATERNION>("diablo_ros_QUATERNION_b", 10);
    STATUSPublisher = n_.advertise<diablo_sdk::OSDK_STATUS>("diablo_ros_STATUS_b", 10);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if (vehicle_->telemetry->newcome & 0x40)
        {
            diablo_sdk::OSDK_STATUS osdk_status_msg;
            osdk_status_msg.ctrl_mode = vehicle_->telemetry->status.ctrl_mode;
            osdk_status_msg.robot_mode = vehicle_->telemetry->status.robot_mode;
            osdk_status_msg.error = vehicle_->telemetry->status.error;
            osdk_status_msg.warning = vehicle_->telemetry->status.warning;
           STATUSPublisher.publish(osdk_status_msg);
            vehicle_->telemetry->eraseNewcomeFlag(0xBF);
        }
        if (vehicle_->telemetry->newcome & 0x20)
        {
            diablo_sdk::OSDK_QUATERNION osdk_quaternion_msg;
            osdk_quaternion_msg.w = vehicle_->telemetry->quaternion.w;
            osdk_quaternion_msg.x = vehicle_->telemetry->quaternion.x;
            osdk_quaternion_msg.y = vehicle_->telemetry->quaternion.y;
            osdk_quaternion_msg.z = vehicle_->telemetry->quaternion.z;
            QUATERNIONPublisher.publish(osdk_quaternion_msg);
            //printf("Quaternion_w:\t%f\nQuaternion_x:\t%f\nQuaternion_y:\t%f\nQuaternion_z:\t%f\n", osdk_quaternion_msg.w, osdk_quaternion_msg.x, osdk_quaternion_msg.y, osdk_quaternion_msg.z);
            vehicle_->telemetry->eraseNewcomeFlag(0xDF);
        }
        if (vehicle_->telemetry->newcome & 0x10)
        {
            diablo_sdk::OSDK_ACCL osdk_accl_msg;
            osdk_accl_msg.x = vehicle_->telemetry->accl.x;
            osdk_accl_msg.y = vehicle_->telemetry->accl.y;
            osdk_accl_msg.z = vehicle_->telemetry->accl.z;
            ACCLPublisher.publish(osdk_accl_msg);
            //printf("ACCL_X:\t%f\nACCL_Y:\t%f\nACCL_Z:\t%f\n", osdk_accl_msg.x, osdk_accl_msg.y, osdk_accl_msg.z);
            vehicle_->telemetry->eraseNewcomeFlag(0xEF);
        }
        if (vehicle_->telemetry->newcome & 0x08)
        {
            diablo_sdk::OSDK_GYRO osdk_gyro_msg;
            osdk_gyro_msg.x = vehicle_->telemetry->gyro.x;
            osdk_gyro_msg.y = vehicle_->telemetry->gyro.y;
            osdk_gyro_msg.z = vehicle_->telemetry->gyro.z;
            GYROPublisher.publish(osdk_gyro_msg);
            //printf("GYRO_X:\t%f\nGYRO_Y:\t%f\nGYRO_Z:\t%f\n", osdk_gyro_msg.x, osdk_gyro_msg.y, osdk_gyro_msg.z);
            vehicle_->telemetry->eraseNewcomeFlag(0xF7);
        }
        if (vehicle_->telemetry->newcome & 0x02)
        {
            diablo_sdk::OSDK_POWER osdk_power_msg;
            osdk_power_msg.battery_voltage = vehicle_->telemetry->power.voltage;
            osdk_power_msg.battery_current = vehicle_->telemetry->power.current;
            osdk_power_msg.battery_capacitor_energy = vehicle_->telemetry->power.capacitor_energy;
            osdk_power_msg.battery_power_percent = vehicle_->telemetry->power.power_percent;
            POWERPublisher.publish(osdk_power_msg);
            //printf("Power:\nVoltage:\t%f\nCurrent:\t%f\nCap_EN:\t%f\nPercent:\t%u\n", osdk_power_msg.battery_voltage, osdk_power_msg.battery_current, osdk_power_msg.battery_current, osdk_power_msg.battery_power_percent);
            vehicle_->telemetry->eraseNewcomeFlag(0xFD);
        }
        if (vehicle_->telemetry->newcome & 0x01)
        {
            diablo_sdk::OSDK_LEGMOTORS osdk_legmotors_msg;
            osdk_legmotors_msg.left_hip_enc_rev = vehicle_->telemetry->motors.left_hip.rev;
            osdk_legmotors_msg.left_hip_pos = vehicle_->telemetry->motors.left_hip.pos;
            osdk_legmotors_msg.left_hip_vel = vehicle_->telemetry->motors.left_hip.vel;
            osdk_legmotors_msg.left_hip_iq = vehicle_->telemetry->motors.left_hip.iq;

            osdk_legmotors_msg.left_knee_enc_rev = vehicle_->telemetry->motors.left_knee.rev;
            osdk_legmotors_msg.left_knee_pos = vehicle_->telemetry->motors.left_knee.pos;
            osdk_legmotors_msg.left_knee_vel = vehicle_->telemetry->motors.left_knee.vel;
            osdk_legmotors_msg.left_knee_iq = vehicle_->telemetry->motors.left_knee.iq;

            osdk_legmotors_msg.left_wheel_enc_rev = vehicle_->telemetry->motors.left_wheel.rev;
            osdk_legmotors_msg.left_wheel_pos = vehicle_->telemetry->motors.left_wheel.pos;
            osdk_legmotors_msg.left_wheel_vel = vehicle_->telemetry->motors.left_wheel.vel;
            osdk_legmotors_msg.left_wheel_iq = vehicle_->telemetry->motors.left_wheel.iq;

            osdk_legmotors_msg.right_hip_enc_rev = vehicle_->telemetry->motors.right_hip.rev;
            osdk_legmotors_msg.right_hip_pos = vehicle_->telemetry->motors.right_hip.pos;
            osdk_legmotors_msg.right_hip_vel = vehicle_->telemetry->motors.right_hip.vel;
            osdk_legmotors_msg.right_hip_iq = vehicle_->telemetry->motors.right_hip.iq;

            osdk_legmotors_msg.right_knee_enc_rev = vehicle_->telemetry->motors.right_knee.rev;
            osdk_legmotors_msg.right_knee_pos = vehicle_->telemetry->motors.right_knee.pos;
            osdk_legmotors_msg.right_knee_vel = vehicle_->telemetry->motors.right_knee.vel;
            osdk_legmotors_msg.right_knee_iq = vehicle_->telemetry->motors.right_knee.iq;

            osdk_legmotors_msg.right_wheel_enc_rev = vehicle_->telemetry->motors.right_wheel.rev;
            osdk_legmotors_msg.right_wheel_pos = vehicle_->telemetry->motors.right_wheel.pos;
            osdk_legmotors_msg.right_wheel_vel = vehicle_->telemetry->motors.right_wheel.vel;
            osdk_legmotors_msg.right_wheel_iq = vehicle_->telemetry->motors.right_wheel.iq;

            LEGMOTORSPublisher.publish(osdk_legmotors_msg);
            vehicle_->telemetry->eraseNewcomeFlag(0xFE);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
}// namespace info_update


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "robot_status_example");
    info_update::InfoUpdate test;
    test.start_robot_status_update();
    return 0;
}

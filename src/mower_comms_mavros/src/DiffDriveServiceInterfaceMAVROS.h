#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/ESCStatus.h>
#include <mower_msgs/ESCStatus.h>
#include <std_msgs/Int32MultiArray.h>

class DiffDriveServiceInterfaceMAVROS {
public:
    DiffDriveServiceInterfaceMAVROS(
        ros::NodeHandle& nh,
        ros::Publisher& actual_twist_pub,
        ros::Publisher& left_esc_pub,
        ros::Publisher& right_esc_pub,
        double ticks_per_meter,
        double wheel_distance);

    void setVelocity(float linear, float angular);
    void stop();

private:
    void escStatusCallback(const mavros_msgs::ESCStatus::ConstPtr& msg);
    void ticksCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void OnWheelTicksChanged(uint32_t left_ticks, uint32_t right_ticks);

    ros::Subscriber esc_status_sub_;
    ros::Subscriber ticks_sub_;

    ros::Publisher actual_twist_pub_;
    ros::Publisher left_esc_pub_;
    ros::Publisher right_esc_pub_;

    mower_msgs::ESCStatus left_esc_state_;
    mower_msgs::ESCStatus right_esc_state_;

    double ticks_per_meter_;
    double wheel_distance_;
};

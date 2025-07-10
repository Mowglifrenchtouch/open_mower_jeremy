#pragma once

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <mower_msgs/Power.h>

class PowerServiceInterfaceMAVROS {
public:
    explicit PowerServiceInterfaceMAVROS(ros::NodeHandle& nh, ros::Publisher& pub);

private:
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    ros::Subscriber battery_sub_;
    ros::Publisher power_pub_;
    mower_msgs::Power power_msg_;
};

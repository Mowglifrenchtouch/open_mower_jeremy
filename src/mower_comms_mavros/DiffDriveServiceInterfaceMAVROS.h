#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class DiffDriveServiceInterfaceMAVROS {
public:
    DiffDriveServiceInterfaceMAVROS(ros::NodeHandle& nh);
    void set_velocity(float linear, float angular);
    void stop();

private:
    ros::Publisher velocity_pub_;
};

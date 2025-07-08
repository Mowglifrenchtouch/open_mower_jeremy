#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class MowerServiceInterfaceMAVROS {
public:
    explicit MowerServiceInterfaceMAVROS(ros::NodeHandle& nh);

    void setEmergency(bool emergency);
    bool isEmergency() const;

private:
    bool emergency_state_;
    ros::Publisher emergency_pub_;
};

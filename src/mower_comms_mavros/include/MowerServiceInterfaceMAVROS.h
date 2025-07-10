#pragma once

#include <ros/ros.h>
#include <mower_msgs/Status.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

class MowerServiceInterfaceMAVROS {
public:
    explicit MowerServiceInterfaceMAVROS(ros::NodeHandle& nh);

    void tick();
    bool isEmergency() const;

private:
    void armedCallback(const std_msgs::Bool::ConstPtr& msg);
    void rpmCallback(const std_msgs::Int32::ConstPtr& msg);
    void escTempCallback(const std_msgs::Float32::ConstPtr& msg);
    void motorTempCallback(const std_msgs::Float32::ConstPtr& msg);
    void currentCallback(const std_msgs::Float32::ConstPtr& msg);
    void rainCallback(const std_msgs::Bool::ConstPtr& msg);

    mower_msgs::Status status_msg_;
    bool emergency_state_;

    ros::Publisher status_pub_;
    ros::Subscriber armed_sub_, rpm_sub_, esc_temp_sub_, motor_temp_sub_, current_sub_, rain_sub_;
};

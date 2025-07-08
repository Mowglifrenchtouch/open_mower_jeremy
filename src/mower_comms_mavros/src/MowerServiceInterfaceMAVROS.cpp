#include "MowerServiceInterfaceMAVROS.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

MowerServiceInterfaceMAVROS::MowerServiceInterfaceMAVROS(ros::NodeHandle& nh)
    : emergency_state_(false)
{
    status_pub_ = nh.advertise<mower_msgs::Status>("ll/status", 1);

    // Subscriptions
    armed_sub_ = nh.subscribe("/mavros/state/armed", 1, &MowerServiceInterfaceMAVROS::armedCallback, this);
    rpm_sub_ = nh.subscribe("/vesc/lame/rpm", 1, &MowerServiceInterfaceMAVROS::rpmCallback, this);
    esc_temp_sub_ = nh.subscribe("/vesc/lame/temperature", 1, &MowerServiceInterfaceMAVROS::escTempCallback, this);
    motor_temp_sub_ = nh.subscribe("/vesc/lame/motor_temp", 1, &MowerServiceInterfaceMAVROS::motorTempCallback, this);
    current_sub_ = nh.subscribe("/vesc/lame/current", 1, &MowerServiceInterfaceMAVROS::currentCallback, this);
    rain_sub_ = nh.subscribe("/sensors/rain", 1, &MowerServiceInterfaceMAVROS::rainCallback, this);
}

void MowerServiceInterfaceMAVROS::armedCallback(const std_msgs::Bool::ConstPtr& msg) {
    status_msg_.mower_enabled = msg->data;
}

void MowerServiceInterfaceMAVROS::rpmCallback(const std_msgs::Int32::ConstPtr& msg) {
    status_msg_.mower_motor_rpm = msg->data;
    status_msg_.mower_running = (std::abs(msg->data) > 100);  // seuil RPM = 100 ?
}

void MowerServiceInterfaceMAVROS::escTempCallback(const std_msgs::Float32::ConstPtr& msg) {
    status_msg_.mower_esc_temp = msg->data;
}

void MowerServiceInterfaceMAVROS::motorTempCallback(const std_msgs::Float32::ConstPtr& msg) {
    status_msg_.mower_motor_temp = msg->data;
}

void MowerServiceInterfaceMAVROS::currentCallback(const std_msgs::Float32::ConstPtr& msg) {
    status_msg_.mower_motor_current = msg->data;
}

void MowerServiceInterfaceMAVROS::rainCallback(const std_msgs::Bool::ConstPtr& msg) {
    status_msg_.rain_detected = msg->data;
}

void MowerServiceInterfaceMAVROS::tick() {
    status_msg_.header.stamp = ros::Time::now();
    status_pub_.publish(status_msg_);
}

bool MowerServiceInterfaceMAVROS::isEmergency() const {
    return emergency_state_;
}

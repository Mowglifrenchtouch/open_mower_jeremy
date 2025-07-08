#include "PowerServiceInterfaceMAVROS.h"

PowerServiceInterfaceMAVROS::PowerServiceInterfaceMAVROS(ros::NodeHandle& nh, ros::Publisher& pub)
    : power_pub_(pub) {
    battery_sub_ = nh.subscribe("/mavros/battery", 10, &PowerServiceInterfaceMAVROS::batteryCallback, this);
}

void PowerServiceInterfaceMAVROS::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    power_msg_.stamp = ros::Time::now();
    power_msg_.v_battery = msg->voltage;
    power_msg_.charge_current = msg->current;
    power_msg_.v_charge = msg->voltage; // ArduPilot ne distingue pas charge vs batterie
    power_msg_.charger_enabled = msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING ? 1 : 0;
    power_msg_.charger_status = msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING ? "charging" : "not charging";

    power_pub_.publish(power_msg_);
}

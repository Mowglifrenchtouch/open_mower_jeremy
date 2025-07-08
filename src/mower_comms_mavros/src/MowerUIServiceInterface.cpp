#include "MowerUIServiceInterface.hpp"

MowerUIServiceInterface::MowerUIServiceInterface(uint16_t service_id,
                                                 const xbot::serviceif::Context& ctx,
                                                 ros::NodeHandle& nh)
    : MowerUIServiceInterfaceBase(service_id, ctx) {
    ui_beep_pub_ = nh.advertise<std_msgs::Bool>("/mower/ui/beep", 1);
    ui_msg_pub_  = nh.advertise<std_msgs::String>("/mower/ui/message", 1);
}

void MowerUIServiceInterface::OnServiceConnected(uint16_t service_id) {
    ROS_INFO("[UI] Service connected: %u", service_id);
}

void MowerUIServiceInterface::OnTransactionStart(uint64_t timestamp) {
    // Nothing to do for now
}

void MowerUIServiceInterface::OnTransactionEnd() {
    // Nothing to do for now
}

void MowerUIServiceInterface::OnMavrosStateChanged(const mavros_msgs::State& state) {
    if (state.mode != last_mode_) {
        last_mode_ = state.mode;
        std_msgs::String msg;
        msg.data = "Flight mode: " + state.mode;
        ui_msg_pub_.publish(msg);
    }
}

void MowerUIServiceInterface::OnBatteryChanged(const sensor_msgs::BatteryState& battery) {
    if (fabs(battery.voltage - last_voltage_) > 0.5f) {
        last_voltage_ = battery.voltage;
        std_msgs::String msg;
        msg.data = "Voltage: " + std::to_string(battery.voltage) + "V";
        ui_msg_pub_.publish(msg);
    }
}

void MowerUIServiceInterface::OnGpsStatusChanged(const mavros_msgs::GPSRAW& gps_status) {
    std_msgs::String msg;
    msg.data = "GPS Sat: " + std::to_string(gps_status.satellites_visible);
    ui_msg_pub_.publish(msg);
}

void MowerUIServiceInterface::OnAltitudeChanged(const std_msgs::Float64& altitude) {
    std_msgs::String msg;
    msg.data = "Alt: " + std::to_string(altitude.data) + "m";
    ui_msg_pub_.publish(msg);
}

void MowerUIServiceInterface::Tick() {
    static ros::Time last_beep_time = ros::Time::now();
    if ((ros::Time::now() - last_beep_time).toSec() > 30.0) {
        std_msgs::Bool beep;
        beep.data = true;
        ui_beep_pub_.publish(beep);
        last_beep_time = ros::Time::now();
    }
}

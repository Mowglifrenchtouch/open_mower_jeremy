#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/GPSRAW.h>
#include <std_msgs/Float64.h>

#include <MowerUIServiceInterfaceBase.hpp>

class MowerUIServiceInterface : public MowerUIServiceInterfaceBase {
public:
    MowerUIServiceInterface(uint16_t service_id,
                            const xbot::serviceif::Context& ctx,
                            ros::NodeHandle& nh);

    void Tick();

protected:
    void OnServiceConnected(uint16_t service_id) override;
    void OnTransactionStart(uint64_t timestamp) override;
    void OnTransactionEnd() override;

    void OnMavrosStateChanged(const mavros_msgs::State& state) override;
    void OnBatteryChanged(const sensor_msgs::BatteryState& battery) override;
    void OnGpsStatusChanged(const mavros_msgs::GPSRAW& gps_status) override;
    void OnAltitudeChanged(const std_msgs::Float64& altitude) override;

private:
    ros::Publisher ui_beep_pub_;
    ros::Publisher ui_msg_pub_;

    std::string last_mode_;
    float last_voltage_ = 0.0;
};

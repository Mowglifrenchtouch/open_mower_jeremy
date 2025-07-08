#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "ImuServiceInterfaceMAVROS.h"
#include "DiffDriveServiceInterfaceMAVROS.h"
#include "EmergencyServiceInterfaceMAVROS.h"
#include "MowerServiceInterfaceMAVROS.h"
#include "GpsServiceInterfaceMAVROS.h"
#include "PowerServiceInterfaceMAVROS.h"

// Global instances
std::shared_ptr<MowerServiceInterfaceMAVROS> mower_service;
std::unique_ptr<EmergencyServiceInterface> emergency_service;
std::unique_ptr<DiffDriveServiceInterfaceMAVROS> diff_drive_service;
std::unique_ptr<GpsServiceInterface> gps_service;
std::unique_ptr<PowerServiceInterfaceMAVROS> power_service;
std::unique_ptr<ImuServiceInterfaceMAVROS> imu_service;

ros::Publisher power_pub;

// RTCM callback
void rtcmReceived(const rtcm_msgs::Message::ConstPtr& msg) {
  ROS_INFO_STREAM_THROTTLE(10.0, "[RTCM] Received " << msg->message.size() << " bytes");
  // gps_service->SendRTCM(msg->message.data(), msg->message.size()); // à activer plus tard
}

void imuReceived(const sensor_msgs::Imu::ConstPtr& msg) {
  if (imu_service) {
    imu_service->processImuMessage(msg);
  }
}
void mowerStatusHeartbeat(const ros::TimerEvent &) {
  if (mower_service) {
    mower_service->tick();
  }
}

// Emergency heartbeat callback
void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent &) {
  if (emergency_service && mower_service) {
    emergency_service->updateEmergencyStatus(mower_service->isEmergency());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_mavros");
  ros::NodeHandle nh;

  ROS_INFO("[MAVROS] Starting mower_comms_mavros node...");

  // Init services
  mower_service = std::make_shared<MowerServiceInterfaceMAVROS>(nh);
  emergency_service = std::make_unique<EmergencyServiceInterface>(mower_service);
  diff_drive_service = std::make_unique<DiffDriveServiceInterfaceMAVROS>(nh);
  gps_service = std::make_unique<GpsServiceInterface>();
  imu_service = std::make_unique<ImuServiceInterfaceMAVROS>(nh);


  // Subscriptions
  ros::Subscriber cmd_vel_sub = nh.subscribe("ll/cmd_vel", 10,
      &DiffDriveServiceInterfaceMAVROS::cmd_vel_callback, diff_drive_service.get());
  ros::Subscriber rtcm_sub = nh.subscribe("ll/position/gps/rtcm", 10, rtcmReceived);
  ros::Subscriber imu_sub = nh.subscribe("mavros/imu/data", 10, imuReceived);


  // Power publishing
  power_pub = nh.advertise<mower_msgs::Power>("ll/power", 1);
  power_service = std::make_unique<PowerServiceInterfaceMAVROS>(nh, power_pub);

  // Emergency heartbeat timer
  ros::Timer emergency_timer = nh.createTimer(ros::Duration(1.0), sendEmergencyHeartbeatTimerTask);
  ros::Timer status_timer = nh.createTimer(ros::Duration(1.0), mowerStatusHeartbeat);


  ROS_INFO("[MAVROS] All interfaces initialized. Spinning...");
  ros::spin();

  return 0;
}

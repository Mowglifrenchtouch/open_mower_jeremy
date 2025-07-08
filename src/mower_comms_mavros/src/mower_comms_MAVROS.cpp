#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/NavSatFix.h>

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

// RTCM callback (placeholder)
void rtcmReceived(const rtcm_msgs::Message::ConstPtr& msg) {
  ROS_INFO_STREAM_THROTTLE(10.0, "[RTCM] Received " << msg->message.size() << " bytes");
  // Future: gps_service->SendRTCM(msg->message.data(), msg->message.size());
}

// Timer callback to refresh emergency status
void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent &) {
  if (emergency_service && mower_service) {
    emergency_service->updateEmergencyStatus(mower_service->isEmergency());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_mavros");
  ros::NodeHandle nh;

  ROS_INFO("[MAVROS] Starting mower_comms_mavros node...");

  // Init MAVROS-based services
  mower_service = std::make_shared<MowerServiceInterfaceMAVROS>(nh);
  emergency_service = std::make_unique<EmergencyServiceInterface>(mower_service);
  diff_drive_service = std::make_unique<DiffDriveServiceInterfaceMAVROS>(nh);
  gps_service = std::make_unique<GpsServiceInterface>();

  // Subscribe to cmd_vel to drive the robot
  ros::Subscriber cmd_vel_sub = nh.subscribe("ll/cmd_vel", 10,
      &DiffDriveServiceInterfaceMAVROS::cmd_vel_callback, diff_drive_service.get());

  // RTCM input (optional)
  ros::Subscriber rtcm_sub = nh.subscribe("ll/position/gps/rtcm", 10, rtcmReceived);

  // Power service via MAVROS
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);
  power_service = std::make_unique<PowerServiceInterfaceMAVROS>(n, power_pub);

  // Emergency heartbeat refresh every 1s
  ros::Timer emergency_timer = nh.createTimer(ros::Duration(1.0), sendEmergencyHeartbeatTimerTask);

  ROS_INFO("[MAVROS] All interfaces initialized. Spinning...");
  ros::spin();

  return 0;
}

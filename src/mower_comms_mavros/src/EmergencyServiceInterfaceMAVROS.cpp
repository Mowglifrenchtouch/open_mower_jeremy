#include "EmergencyServiceInterfaceMAVROS.h"

EmergencyServiceInterfaceMAVROS::EmergencyServiceInterfaceMAVROS(ros::NodeHandle& nh) {
  publisher_ = nh.advertise<mower_msgs::Emergency>("ll/emergency", 10, true);
}

bool EmergencyServiceInterfaceMAVROS::setEmergency(bool new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  active_high_level_emergency_ = new_value;
  if (new_value) {
    latest_emergency_reason_ = "High Level Emergency (manual)";
  }
  publishEmergencyState();
  return true;
}

void EmergencyServiceInterfaceMAVROS::heartbeat() {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::onEmergencyActiveChanged(bool new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  active_low_level_emergency_ = new_value;
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::onEmergencyLatchChanged(bool new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  if (new_value) {
    latched_emergency_ = true;
  } else if (!active_high_level_emergency_) {
    latched_emergency_ = false;
  }
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::onEmergencyReasonChanged(const std::string& reason) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  latest_emergency_reason_ = reason;
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::onServiceConnected() {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  latched_emergency_ = active_low_level_emergency_ = active_high_level_emergency_ = true;
  latest_emergency_reason_ = "Service Connected (MAVROS)";
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::onServiceDisconnected() {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  latched_emergency_ = active_low_level_emergency_ = active_high_level_emergency_ = true;
  latest_emergency_reason_ = "Service Disconnected";
  publishEmergencyState();
}

void EmergencyServiceInterfaceMAVROS::publishEmergencyState() {
  mower_msgs::Emergency msg;
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  msg.stamp = ros::Time::now();
  msg.latched_emergency = latched_emergency_ |= active_high_level_emergency_ | active_low_level_emergency_;
  msg.active_emergency = active_high_level_emergency_ | active_low_level_emergency_;
  msg.reason = latest_emergency_reason_;
  publisher_.publish(msg);
}

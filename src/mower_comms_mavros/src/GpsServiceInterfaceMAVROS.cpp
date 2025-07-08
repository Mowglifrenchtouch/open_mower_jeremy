#include "service_ids.h"
#include "GpsServiceInterfaceMAVROS.h"

GpsServiceInterface::GpsServiceInterface()
  : latitude_(0.0), longitude_(0.0), altitude_(0.0) {
  gps_sub_ = nh_.subscribe("mavros/global_position/global", 10, &GpsServiceInterface::gps_callback, this);
}

void GpsServiceInterface::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  latitude_ = msg->latitude;
  longitude_ = msg->longitude;
  altitude_ = msg->altitude;
}

double GpsServiceInterface::get_latitude() {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  return latitude_;
}

double GpsServiceInterface::get_longitude() {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  return longitude_;
}

double GpsServiceInterface::get_altitude() {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  return altitude_;
}

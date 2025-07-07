#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mutex>

class GpsServiceInterface {
public:
  GpsServiceInterface();

  double get_latitude();
  double get_longitude();
  double get_altitude();

private:
  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;

  std::mutex gps_mutex_;
  double latitude_;
  double longitude_;
  double altitude_;
};

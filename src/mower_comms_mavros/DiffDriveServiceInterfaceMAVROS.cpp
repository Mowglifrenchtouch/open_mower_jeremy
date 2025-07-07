#include "DiffDriveServiceInterfaceMAVROS.h"

DiffDriveServiceInterfaceMAVROS::DiffDriveServiceInterfaceMAVROS(ros::NodeHandle& nh) {
    // Publie sur le topic MAVROS qui contrôle la vitesse (à adapter selon config)
    velocity_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void DiffDriveServiceInterfaceMAVROS::set_velocity(float linear, float angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    velocity_pub_.publish(cmd);
}

void DiffDriveServiceInterfaceMAVROS::stop() {
    set_velocity(0.0f, 0.0f);
}

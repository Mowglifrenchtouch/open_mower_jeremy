#include "service_ids.h"
#include "MowerServiceInterfaceMAVROS.h"

MowerServiceInterfaceMAVROS::MowerServiceInterfaceMAVROS(ros::NodeHandle& nh)
    : emergency_state_(false)
{
    // Correction ici : topic MAVROS conventionnel
    emergency_pub_ = nh.advertise<std_msgs::Bool>("ll/emergency", 1, true);
}

void MowerServiceInterfaceMAVROS::setEmergency(bool emergency)
{
    emergency_state_ = emergency;

    std_msgs::Bool msg;
    msg.data = emergency;
    emergency_pub_.publish(msg);

    ROS_INFO_STREAM("[MAVROS] Emergency status set to: " << (emergency ? "TRUE" : "FALSE"));
}

bool MowerServiceInterfaceMAVROS::isEmergency() const
{
    return emergency_state_;
}

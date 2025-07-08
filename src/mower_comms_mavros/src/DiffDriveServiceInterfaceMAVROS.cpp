#include "DiffDriveServiceInterfaceMAVROS.h"

DiffDriveServiceInterfaceMAVROS::DiffDriveServiceInterfaceMAVROS(
    ros::NodeHandle& nh,
    ros::Publisher& actual_twist_pub,
    ros::Publisher& left_esc_pub,
    ros::Publisher& right_esc_pub)
    : actual_twist_pub_(actual_twist_pub),
      left_esc_pub_(left_esc_pub),
      right_esc_pub_(right_esc_pub) {

    nh.param("ticks_per_meter", ticks_per_meter_, 1000.0);  // configurable
    nh.param("wheel_distance", wheel_distance_, 0.45);      // configurable

    esc_status_sub_ = nh.subscribe("/mavros/esc_status", 10,
                                   &DiffDriveServiceInterfaceMAVROS::escStatusCallback, this);
    ticks_sub_ = nh.subscribe("/vesc/ticks", 10,
                              &DiffDriveServiceInterfaceMAVROS::ticksCallback, this);
}

void DiffDriveServiceInterfaceMAVROS::setVelocity(float linear, float angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    actual_twist_pub_.publish(cmd);
}

void DiffDriveServiceInterfaceMAVROS::stop() {
    setVelocity(0.0f, 0.0f);
}

void DiffDriveServiceInterfaceMAVROS::escStatusCallback(const mavros_msgs::ESCStatus::ConstPtr& msg) {
    if (msg->esc_status.size() < 2) return;

    static uint32_t seq = 0;
    ros::Time now = ros::Time::now();

    const auto& esc_left = msg->esc_status[0];
    const auto& esc_right = msg->esc_status[1];

    left_esc_state_.header.stamp = now;
    left_esc_state_.header.seq = seq;
    left_esc_state_.header.frame_id = "left_motor";
    left_esc_state_.temperature = esc_left.temperature;
    left_esc_state_.current = esc_left.current;
    left_esc_state_.status = esc_left.rpm > 0 ? 1 : 0;

    right_esc_state_.header.stamp = now;
    right_esc_state_.header.seq = seq++;
    right_esc_state_.header.frame_id = "right_motor";
    right_esc_state_.temperature = esc_right.temperature;
    right_esc_state_.current = esc_right.current;
    right_esc_state_.status = esc_right.rpm > 0 ? 1 : 0;

    left_esc_pub_.publish(left_esc_state_);
    right_esc_pub_.publish(right_esc_state_);
}

void DiffDriveServiceInterfaceMAVROS::OnWheelTicksChanged(uint32_t left_ticks, uint32_t right_ticks) {
    static uint32_t prev_left_ticks = 0;
    static uint32_t prev_right_ticks = 0;

    if (abs((int32_t)left_ticks - (int32_t)prev_left_ticks) > 10000 ||
        abs((int32_t)right_ticks - (int32_t)prev_right_ticks) > 10000)
        return;

    double delta_left = static_cast<double>(left_ticks - prev_left_ticks) / ticks_per_meter_;
    double delta_right = static_cast<double>(right_ticks - prev_right_ticks) / ticks_per_meter_;

    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;

    double linear = (delta_left + delta_right) / 2.0;
    double angular = (delta_right - delta_left) / wheel_distance_;

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear;
    twist_msg.angular.z = angular;
    actual_twist_pub_.publish(twist_msg);

    ROS_DEBUG("WheelTicks updated: left=%u, right=%u -> linear=%.3f m/s, angular=%.3f rad/s",
              left_ticks, right_ticks, linear, angular);
}

void DiffDriveServiceInterfaceMAVROS::ticksCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        uint32_t left_ticks = static_cast<uint32_t>(msg->data[0]);
        uint32_t right_ticks = static_cast<uint32_t>(msg->data[1]);
        OnWheelTicksChanged(left_ticks, right_ticks);
    }
}
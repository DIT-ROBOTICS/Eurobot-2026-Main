#ifndef LOC_RECEIVER_HPP
#define LOC_RECEIVER_HPP

#include "receiver_util.hpp"

// receive the robot pose and rival pose from localization team

class LocReceiver
{
public:
    LocReceiver(const RosNodeParams& params)
        : node_(params.nh.lock()), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
    {
        node_->get_parameter("frame_id", frame_id_);
    }
    static bool UpdateRobotPose(geometry_msgs::msg::PoseStamped &robot_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_);
    static bool UpdateRivalPose(geometry_msgs::msg::PoseStamped &rival_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_);

private:
    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    std::string frame_id_;

    geometry_msgs::msg::PoseStamped robot_pose_;
    geometry_msgs::msg::PoseStamped rival_pose_;
};
#endif // LOC_RECEIVER_HPP
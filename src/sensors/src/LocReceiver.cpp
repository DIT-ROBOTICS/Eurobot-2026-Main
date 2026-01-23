#include "LocReceiver.hpp"

bool LocReceiver::UpdateRobotPose(geometry_msgs::msg::PoseStamped &robot_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "map", 
            frame_id_,
            rclcpp::Time()
        );
        robot_pose_.pose.position.x = transformStamped.transform.translation.x;
        robot_pose_.pose.position.y = transformStamped.transform.translation.y;
        robot_pose_.pose.position.z = 0;
        robot_pose_.pose.orientation = transformStamped.transform.rotation;
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("localization"), "[UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
        return false;
    }
}

bool LocReceiver::UpdateRivalPose(geometry_msgs::msg::PoseStamped &rival_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "map" /* Parent frame - map */, 
            "rival/" + frame_id_ /* Child frame - base */,
            rclcpp::Time()
        );
        rival_pose_.pose.position.x = transformStamped.transform.translation.x;
        rival_pose_.pose.position.y = transformStamped.transform.translation.y;
        rival_pose_.pose.position.z = 0;
        rival_pose_.pose.orientation = transformStamped.transform.rotation;
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("localization"), "[UpdateRivalPose]: line " << __LINE__ << " " << ex.what());
        return false;
    }
}
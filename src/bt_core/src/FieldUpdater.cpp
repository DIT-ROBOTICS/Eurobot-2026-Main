#include "FieldUpdater.hpp"

FieldUpdater::FieldUpdater(const std::string& name, const BT::NodeConfig& config, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard) :
    BT::SyncActionNode(name, config), 
    node_(params.nh.lock()),
    blackboard_ptr(blackboard) {
    
    // Initialize publisher for keepout zones
    keepout_zone_pub = node_->create_publisher<std_msgs::msg::String>("/keepout_zone", 10);
    
    readBlackboard();
}

BT::PortsList FieldUpdater::providedPorts() {
    return {};
}

BT::NodeStatus FieldUpdater::tick() {
    readBlackboard();
    updateFieldStatus();
    return BT::NodeStatus::SUCCESS;
}

void FieldUpdater::readBlackboard() {
    if (!blackboard_ptr->get<std_msgs::msg::Int32MultiArray>("collection_info_raw", collection_info_raw)) {
        RCLCPP_WARN(node_->get_logger(), "[FieldUpdater] collection_info_raw not found in blackboard");
    }
    if (!blackboard_ptr->get<std_msgs::msg::Int32MultiArray>("pantry_info_raw", pantry_info_raw)) {
        RCLCPP_WARN(node_->get_logger(), "[FieldUpdater] pantry_info_raw not found in blackboard");
    }
}

void FieldUpdater::updateFieldStatus() {
    std::string keepout_zone_msg = "";
    for(size_t i = 0; i < collection_info_raw.data.size(); i++) {
        int val = collection_info_raw.data[i];
        if (val != 0) { // -1 (blocked), 1, 2, 3 (occupied) -> add to keepout
            keepout_zone_msg += goalPoseToString(static_cast<GoalPose>(static_cast<int>(GoalPose::K) + i));
        }
    }

    for(size_t i = 0; i < pantry_info_raw.data.size(); i++) {
        int val = pantry_info_raw.data[i];
        if (val != 0) { // -1 (blocked), 1, 2, 3 (occupied) -> add to keepout
            keepout_zone_msg += goalPoseToString(static_cast<GoalPose>(static_cast<int>(GoalPose::A) + i));
        }
    }
    RCLCPP_DEBUG(node_->get_logger(), "[FieldUpdater] Keepout zone: %s", keepout_zone_msg.c_str());
    
    auto msg = std_msgs::msg::String();
    msg.data = keepout_zone_msg;
    keepout_zone_pub->publish(msg);
}
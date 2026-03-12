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
    if (!blackboard_ptr->get<vector<FieldStatus>>("collection_info", collection_info)) {
        RCLCPP_WARN(node_->get_logger(), "[FieldUpdater] collection_info not found in blackboard");
    }
    if (!blackboard_ptr->get<vector<FieldStatus>>("pantry_info", pantry_info)) {
        RCLCPP_WARN(node_->get_logger(), "[FieldUpdater] pantry_info not found in blackboard");
    }
}

void FieldUpdater::updateFieldStatus() {
    // TODO: implement field status update logic
    std::string keepout_zone_msg = "";
    for(size_t i = 0; i < collection_info.size(); i++) {
        if (collection_info[i] == FieldStatus::OCCUPIED) {
            keepout_zone_msg += goalPoseToString(static_cast<GoalPose>(static_cast<int>(GoalPose::K) + i));
        }
    }

    for(size_t i = 0; i < pantry_info.size(); i++) {
        if (pantry_info[i] == FieldStatus::OCCUPIED) {
            keepout_zone_msg += goalPoseToString(static_cast<GoalPose>(static_cast<int>(GoalPose::A) + i));
        }
    }
    RCLCPP_INFO(node_->get_logger(), "[FieldUpdater] Keepout zone: %s", keepout_zone_msg.c_str());
    
    auto msg = std_msgs::msg::String();
    msg.data = keepout_zone_msg;
    keepout_zone_pub->publish(msg);
}
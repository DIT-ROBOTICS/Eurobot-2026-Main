#include "FieldUpdater.hpp"

FieldUpdater::FieldUpdater(const std::string& name, const BT::NodeConfig& config, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard) :
    BT::SyncActionNode(name, config), blackboard_ptr(blackboard) {
    
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
    blackboard_ptr->get<vector<FieldStatus>>("collection_info", collection_info);
    blackboard_ptr->get<vector<FieldStatus>>("pantry_info", pantry_info);
}

void FieldUpdater::updateFieldStatus() {
    // TODO: implement field status update logic
    std::string keepout_zone_msg = "";
    for(int i = 0; i < collection_info.size(); i++) {
        if (collection_info[i].status == FieldStatus::OCCUPIED) {
            keepout_zone_msg += goalPoseToString(GoalPose::K + i);
        }
    }

    for(int i = 0; i < pantry_info.size(); i++) {
        if (pantry_info[i].status == FieldStatus::OCCUPIED) {
            keepout_zone_msg += goalPoseToString(GoalPose::A + i);
        }
    }
    RCLCPP_INFO(get_logger(), "[FieldUpdater] Keepout zone: %s", keepout_zone_msg.c_str());
    keepout_zone_pub->publish(keepout_zone_msg);
}
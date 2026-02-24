#ifndef FIELD_UPDATER_HPP
#define FIELD_UPDATER_HPP

#include "bt_config.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

using namespace BT;
using namespace std;

class FieldUpdater : public BT::SyncActionNode {
public:
    FieldUpdater(const std::string& name, const BT::NodeConfig& config, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    
private:
    
    void readBlackboard();
    void updateFieldStatus();
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keepout_zone_pub;
    BT::Blackboard::Ptr blackboard_ptr;
    vector<FieldStatus> collection_info;
    vector<FieldStatus> pantry_info;
};
#include "CamReceiver.hpp"

CamReceiver::CamReceiver(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard){

    collection_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "collection_status", 10, std::bind(&CamReceiver::collection_info_callback, this, std::placeholders::_1));
    pantry_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "pantry_status", 10, std::bind(&CamReceiver::pantry_info_callback, this, std::placeholders::_1));
}

PortsList CamReceiver::providedPorts() {
    return {};
}

void CamReceiver::collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // TODO: delete debug info log
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Received collection info update.");
    collection_info = *msg;
    if (!collection_info.data.empty()) blackboard_->set<std_msgs::msg::Int32MultiArray>("collection_info", collection_info);
}

void CamReceiver::pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    // TODO: delete debug info log
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Received pantry info update.");
    pantry_info = *msg;
    if (!pantry_info.data.empty()) blackboard_->set<std_msgs::msg::Int32MultiArray>("pantry_info", pantry_info);
}

BT::NodeStatus CamReceiver::tick() {
    RCLCPP_INFO(node_->get_logger(), "CamReceiver start");
    // TODO: delete debug info log
    for (size_t i = 0; i < collection_info.data.size(); ++i) 
        RCLCPP_INFO(node_->get_logger(), "Collection Info [%zu]: %d", i, collection_info.data[i]);

    for (size_t i = 0; i < pantry_info.data.size(); ++i) 
        RCLCPP_INFO(node_->get_logger(), "Pantry Info [%zu]: %d", i, pantry_info.data[i]);
    
    
    return BT::NodeStatus::SUCCESS;
}
#ifndef CONTROLLER_TYPE_PUBLISHER_HPP
#define CONTROLLER_TYPE_PUBLISHER_HPP

#include "navigation_util.hpp"

class ControllerTypePublisher : public BT::SyncActionNode {

public:
    ControllerTypePublisher(const std::string& name, const NodeConfig& config,
                            const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void readBlackboard();

    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std::string controller_type_info_;
    std::string mode_;
    std_msgs::msg::Int32MultiArray collection_info;
    std_msgs::msg::Int32MultiArray pantry_info;
};

#endif // CONTROLLER_TYPE_PUBLISHER_HPP

#ifndef BT_STOP_ROBOT_NODE_HPP
#define BT_STOP_ROBOT_NODE_HPP
#include "navigation_util.hpp"
class StopRobotNode : public BT::SyncActionNode {

public:
    StopRobotNode(const std::string& name, const NodeConfig& config, const RosNodeParams& params);

    static PortsList providedPorts();

    NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> node;
    BT::Blackboard::Ptr blackboard;
    rclcpp::Rate rate;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
    std_msgs::msg::Bool stop_msg;
};

#endif // BT_STOP_ROBOT_NODE_HPP
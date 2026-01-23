#include "stop_robot_node.hpp"

StopRobotNode::StopRobotNode(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    : BT::SyncActionNode(name, config), node(params.nh.lock()), rate(30) {
    stop_pub = node->create_publisher<std_msgs::msg::Bool>("/stopRobot", rclcpp::QoS(10).reliable().transient_local());
}

PortsList StopRobotNode::providedPorts() {
    return {};
}

BT::NodeStatus StopRobotNode::tick() {
    stop_msg.data = true;
    for (int i = 0; i < 10; i++) {
        stop_pub->publish(stop_msg);
        rate.sleep();
    }
    stop_msg.data = false;
    for (int i = 0; i < 10; i++) {
        stop_pub->publish(stop_msg);
        rate.sleep();
    }
    return BT::NodeStatus::SUCCESS;
}
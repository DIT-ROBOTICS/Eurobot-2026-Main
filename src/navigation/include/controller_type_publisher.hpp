#ifndef CONTROLLER_TYPE_PUBLISHER_HPP
#define CONTROLLER_TYPE_PUBLISHER_HPP

#include "navigation_util.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <chrono>
#include <thread>

class ControllerTypePublisher : public BT::SyncActionNode {

public:
    ControllerTypePublisher(const std::string& name, const NodeConfig& config,
                            const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void readBlackboard();
    void publishControllerType();
    void spinThread();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_type_pub_;
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std::string controller_type_info;
    std_msgs::msg::Int32MultiArray collection_info;
    std_msgs::msg::Int32MultiArray pantry_info;
};

#endif // CONTROLLER_TYPE_PUBLISHER_HPP

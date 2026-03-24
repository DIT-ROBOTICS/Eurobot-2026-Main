#include "CursorPublisher.hpp"

// BLUE colored logs for CursorPublisher
#define CP_COLOR "\033[34m"  // Blue
#define CP_RESET "\033[0m"
#define CP_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), CP_COLOR "[CursorPublisher] " fmt CP_RESET, ##__VA_ARGS__)
#define CP_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), CP_COLOR "[CursorPublisher] " fmt CP_RESET, ##__VA_ARGS__)

CursorPublisher::CursorPublisher(const std::string& name, const BT::NodeConfig& config,
                                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard) {
    left_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/on_cursor_left", 10);
    right_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/on_cursor_right", 10);
    CP_INFO(node_, "Initialized — publishing to /robot/on_cursor_left and /robot/on_cursor_right");
}

BT::PortsList CursorPublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("direction", "left", "Direction: 'left' or 'right'"),
        BT::InputPort<bool>("value", false, "Bool value to publish")
    };
}

BT::NodeStatus CursorPublisher::onStart() {
    // Read input ports
    std::string direction;
    if (!getInput<std::string>("direction", direction)) {
        CP_WARN(node_, "Missing direction, defaulting to 'left'");
        direction = "left";
    }

    bool value;
    if (!getInput<bool>("value", value)) {
        CP_WARN(node_, "Missing value, defaulting to false");
        value = false;
    }

    // Create message
    std_msgs::msg::Bool msg;
    msg.data = value;

    // Publish based on direction
    if (direction == "left") {
        CP_INFO(node_, "Publishing to /robot/on_cursor_left: %s", value ? "true" : "false");
        left_pub_->publish(msg);
    } else if (direction == "right") {
        CP_INFO(node_, "Publishing to /robot/on_cursor_right: %s", value ? "true" : "false");
        right_pub_->publish(msg);
    } else {
        CP_WARN(node_, "Invalid direction '%s', not publishing", direction.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CursorPublisher::onRunning() {
    // Immediately succeed after publishing
    return BT::NodeStatus::SUCCESS;
}

void CursorPublisher::onHalted() {
    CP_INFO(node_, "Halted (preempted)");
}

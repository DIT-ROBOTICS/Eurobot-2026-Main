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
        BT::InputPort<std::string>("arms", "left", "arms: 'left' or 'right'"),
        BT::InputPort<bool>("value", false, "Bool value to publish"),
    };
}

BT::NodeStatus CursorPublisher::onStart() {
    // Read input ports
    if (!getInput<std::string>("arms", arms_)) {
        CP_WARN(node_, "Missing arms, defaulting to 'left'");
        arms_ = "left";
    }

    if (!getInput<bool>("value", value_)) {
        CP_WARN(node_, "Missing value, defaulting to false");
        value_ = false;
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CursorPublisher::onRunning() {
    // Immediately succeed after publishing
    if(isArriveGoalPose()){
        // Create message
        std_msgs::msg::Bool msg;
        msg.data = value_;

        // Publish based on arms
        if (arms_ == "left") {
            CP_INFO(node_, "Publishing to /robot/on_cursor_left: %s", value_ ? "true" : "false");
            left_pub_->publish(msg);
        } else if (arms_ == "right") {
            CP_INFO(node_, "Publishing to /robot/on_cursor_right: %s", value_ ? "true" : "false");
            right_pub_->publish(msg);
        } else {
            CP_WARN(node_, "Invalid arms '%s', not publishing", arms_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void CursorPublisher::onHalted() {
    CP_INFO(node_, "Halted (preempted)");
}

bool CursorPublisher::isArriveGoalPose()
{
    geometry_msgs::msg::PoseStamped robot_pose_;

    if (!blackboard_->get("robot_pose", robot_pose_)) {
        CP_WARN(node_, "No robot_pose in blackboard");
        return false;
    }

    double y = robot_pose_.pose.position.y;

    // CP_INFO(node_, "Current y: %.3f", y);

    const double target_y = 0.2;
    const double tolerance = 0.15;

    return std::abs(y - target_y) < tolerance;
}
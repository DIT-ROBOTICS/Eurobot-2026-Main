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
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index (for collection update)"),
    };
}

BT::NodeStatus CursorPublisher::onStart() {
    // Read input ports
    if (!getInput<std::string>("arms", arms_)) {
        CP_WARN(node_, "Missing arms, defaulting to 'left'");
        arms_ = "left";
    }

    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        CP_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }

    if (!blackboard_->get<std::vector<MapPoint>>("MapPointList", map_point_list_)) {
        CP_WARN(node_, "[OnDockAction] Failed to get MapPointList from blackboard");
    }

    is_arm_on_ = 0;
    cursor_state_ = 0;
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CursorPublisher::onRunning() {
    // Immediately succeed after publishing
    checkPosition()
    std_msgs::msg::Bool msg;
    switch (cursor_state_) {
        case 1:
            if (is_arm_on_ == 1) break;
            msg.data = true; // aligned in y, can use arm
            if (arms_ == "left") {
                CP_INFO(node_, "Publishing to /robot/on_cursor_left: 1");
                left_pub_->publish(msg);
            } else if (arms_ == "right") {
                CP_INFO(node_, "Publishing to /robot/on_cursor_right: 1");
                right_pub_->publish(msg);
            }
            is_arm_on_ = 1;
            return BT::NodeStatus::RUNNING;
        case 2:
            msg.data = false; // fully aligned, finish cursor action
            CP_INFO(node_, "Fully aligned with target pose %d", target_pose_idx_);
            if (arms_ == "left") {
                CP_INFO(node_, "Publishing to /robot/on_cursor_left: 0");
                left_pub_->publish(msg);
            } else if (arms_ == "right") {
                CP_INFO(node_, "Publishing to /robot/on_cursor_right: 0");
                right_pub_->publish(msg);
            }
            is_arm_on_ = 0;
            return BT::NodeStatus::SUCCESS;
        default:
            break;
    }
    return BT::NodeStatus::RUNNING;
}

void CursorPublisher::onHalted() {
    CP_INFO(node_, "Halted (preempted)");
}

int CursorPublisher::checkPosition()
{
    geometry_msgs::msg::PoseStamped robot_pose_;

    if (!blackboard_->get("robot_pose", robot_pose_)) {
        CP_WARN(node_, "No robot_pose in blackboard");
        return false;
    }

    double x_ = robot_pose_.pose.position.x;
    double y_ = robot_pose_.pose.position.y;

    double target_x = map_point_list_[target_pose_idx_].x;
    double target_y = map_point_list_[target_pose_idx_].y;

    if (std::abs(x_ - target_x) < tolerance_ && std::abs(y_ - target_y) < tolerance_) {
        cursor_state_ = 2; // fully aligned
        return 2;
    } else if (std::abs(y_ - target_y) < tolerance_) {
        cursor_state_ = 1; // y aligned
        return 1;
    }
    return 0;
}
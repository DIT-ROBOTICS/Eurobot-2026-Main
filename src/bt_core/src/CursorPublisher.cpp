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

    if (!node_->has_parameter("cursor_tolerance")) node_->declare_parameter("cursor_tolerance", 0.18);
    node_->get_parameter("cursor_tolerance", tolerance_);

    blackboard_->set("left_cursor_status", std::make_pair(std::string("left"), false));
    blackboard_->set("right_cursor_status", std::make_pair(std::string("right"), false));
}

BT::PortsList CursorPublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("arms", "left", "arms: 'left' or 'right'"),
        BT::InputPort<std::string>("state", "on", "state: 'on' or 'off'"),
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index (for collection update)"),
    };
}

BT::NodeStatus CursorPublisher::onStart() {
    // Read input ports
    if (!getInput<std::string>("arms", arms_)) {
        CP_WARN(node_, "Missing arms, defaulting to 'left'");
        arms_ = "left";
    }

    if (!getInput<std::string>("state", state_)) {
        CP_WARN(node_, "Missing state, defaulting to 'on'");
        state_ = "on";
    }

    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        CP_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }

    if (!blackboard_->get<std::vector<MapPoint>>("MapPointList", map_point_list_)) {
        CP_WARN(node_, "Failed to get MapPointList from blackboard");
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CursorPublisher::onRunning() {
    std_msgs::msg::Bool msg;

    if (state_ == "off") {
        msg.data = false;
        if (arms_ == "left") {
            CP_INFO(node_, "Publishing to /robot/on_cursor_left: 0");
            left_pub_->publish(msg);
            blackboard_->set("left_cursor_status", std::make_pair("left", false));
        }
        else if (arms_ == "right") {
            CP_INFO(node_, "Publishing to /robot/on_cursor_right: 0");
            right_pub_->publish(msg);
            blackboard_->set("right_cursor_status", std::make_pair("right", false));
        }
        else {
            CP_WARN(node_, "Invalid arms: %s", arms_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    if (state_ != "on") {
        CP_WARN(node_, "Invalid state: %s (expected 'on' or 'off')", state_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // state == on: only publish when y-stage aligned
    if (checkPosition()) {
        msg.data = true;
        if (arms_ == "left") {
            CP_INFO(node_, "Y aligned, publishing to /robot/on_cursor_left: 1");
            left_pub_->publish(msg);
            blackboard_->set("left_cursor_status", std::make_pair("left", true));
        }
        else if (arms_ == "right") {
            CP_INFO(node_, "Y aligned, publishing to /robot/on_cursor_right: 1");
            right_pub_->publish(msg);
            blackboard_->set("right_cursor_status", std::make_pair("right", true));
        }
        else {
            CP_WARN(node_, "Invalid arms: %s", arms_.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void CursorPublisher::onHalted() {
    CP_INFO(node_, "Halted (preempted)");
}

bool CursorPublisher::checkPosition()
{
    geometry_msgs::msg::PoseStamped robot_pose_;

    if (!blackboard_->get("robot_pose", robot_pose_)) {
        CP_WARN(node_, "No robot_pose in blackboard");
        return false;
    }

    if (target_pose_idx_ < 0 || target_pose_idx_ >= static_cast<int>(map_point_list_.size())) {
        CP_WARN(node_, "Invalid targetPoseIdx: %d", target_pose_idx_);
        return false;
    }

    double x_ = robot_pose_.pose.position.x;
    double y_ = robot_pose_.pose.position.y;

    double target_x = map_point_list_[target_pose_idx_].x;
    double target_y = map_point_list_[target_pose_idx_].y;

    double stage_point = map_point_list_[target_pose_idx_].staging_dist;
    double docking_sign = map_point_list_[target_pose_idx_].sign;

    return std::abs(x_ - (target_x + stage_point * docking_sign)) < tolerance_ && std::abs(y_ - target_y) < tolerance_;
}
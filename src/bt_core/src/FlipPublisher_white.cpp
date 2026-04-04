#include "FlipPublisher_white.hpp"
#include <cmath>

// BRIGHT GREEN colored logs for FlipPublisher_white
#define FPW_COLOR "\033[92m"  // Bright Green
#define FPW_RESET "\033[0m"
#define FPW_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), FPW_COLOR "[FlipPublisher_white] " fmt FPW_RESET, ##__VA_ARGS__)
#define FPW_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), FPW_COLOR "[FlipPublisher_white] " fmt FPW_RESET, ##__VA_ARGS__)

FlipPublisher_white::FlipPublisher_white(const std::string& name, const BT::NodeConfig& config,
                                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      side_idx_(0),
      target_pose_idx_(0),
      timeout_ms_(5000),
      flip_distance_threshold_(0.15),
      start_published_(false) {
    flip_start_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/robot/on_flip", 10);

    if (!node_->has_parameter("flip_distance_threshold")) {
        node_->declare_parameter("flip_distance_threshold", 0.15);
    }
    flip_distance_threshold_ = node_->get_parameter("flip_distance_threshold").as_double();

    FPW_INFO(node_, "Initialized — distance threshold: %.2fm, publishing to /robot/on_flip", flip_distance_threshold_);
}

BT::PortsList FlipPublisher_white::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to use"),
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index in map_points"),
        BT::InputPort<int>("timeout_ms", 5000, "Timeout in milliseconds")
    };
}

BT::NodeStatus FlipPublisher_white::onStart() {
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        FPW_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        FPW_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        timeout_ms_ = 5000;
    }

    readBlackboard();

    start_published_ = false;
    start_time_ = std::chrono::steady_clock::now();

    FPW_INFO(node_, "Waiting to reach distance %.2f to target idx %d (timeout: %d ms)",
             flip_distance_threshold_, target_pose_idx_, timeout_ms_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FlipPublisher_white::onRunning() {
    if (start_published_) {
        return BT::NodeStatus::SUCCESS;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!blackboard_->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        FPW_WARN(node_, "Failed to get robot_pose from blackboard, will rely on timeout.");
    } else {
        if (target_pose_idx_ < static_cast<int>(map_point_list_.size())) {
            double target_x = map_point_list_[target_pose_idx_].x;
            double target_y = map_point_list_[target_pose_idx_].y;
            double dist = std::hypot(robot_pose.pose.position.x - target_x,
                                     robot_pose.pose.position.y - target_y);

            if (dist <= flip_distance_threshold_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time_).count();
                FPW_INFO(node_, "Reached distance limit (%.3f <= %.3f)! Publishing FLIP start for side %d (waited %ld ms)",
                         dist, flip_distance_threshold_, side_idx_, elapsed);
                publishFlipStart(side_idx_);
                start_published_ = true;
                return BT::NodeStatus::SUCCESS;
            }
        } else {
            FPW_WARN(node_, "Invalid map_points index %d or array length %zu",
                     target_pose_idx_, map_point_list_.size());
        }
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= timeout_ms_) {
        FPW_WARN(node_, "Timeout (%d ms) — publishing FLIP start anyway for side %d", timeout_ms_, side_idx_);
        publishFlipStart(side_idx_);
        start_published_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void FlipPublisher_white::onHalted() {
    FPW_INFO(node_, "Halted (preempted)");
}

void FlipPublisher_white::readBlackboard() {
    if (!blackboard_->get<std::vector<MapPoint>>("MapPointList", map_point_list_)) {
        FPW_WARN(node_, "MapPointList not in blackboard");
    }
}

void FlipPublisher_white::publishFlipStart(int side_idx) {
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        FPW_WARN(node_, "Invalid side index: %d", side_idx);
        return;
    }

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(side_idx);
    FPW_INFO(node_, "Publishing FLIP start for side %d", side_idx);
    flip_start_pub_->publish(msg);
}

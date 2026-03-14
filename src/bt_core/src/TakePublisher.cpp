#include "TakePublisher.hpp"

// CYAN colored logs for TakePublisher
#define TP_COLOR "\033[96m"  // Cyan
#define TP_RESET "\033[0m"
#define TP_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), TP_COLOR "[TakePublisher] " fmt TP_RESET, ##__VA_ARGS__)
#define TP_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), TP_COLOR "[TakePublisher] " fmt TP_RESET, ##__VA_ARGS__)
#define TP_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), TP_COLOR "[TakePublisher] " fmt TP_RESET, ##__VA_ARGS__)

TakePublisher::TakePublisher(const std::string& name, const BT::NodeConfig& config,
                             const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      side_idx_(0),
      target_pose_idx_(0),
      timeout_ms_(3000),
      take_published_(false) {
    take_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/robot/on_take", 10);
    TP_INFO(node_, "Initialized — publishing to /robot/on_take");
}

BT::PortsList TakePublisher::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to use"),
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index (for collection update)"),
        BT::InputPort<int>("timeout_ms", 3000, "Timeout in milliseconds")
    };
}

BT::NodeStatus TakePublisher::onStart() {
    // Read input ports
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        TP_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        TP_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        timeout_ms_ = 3000;
    }

    // Read blackboard
    readBlackboard();

    // Publish take command once
    publishTake(side_idx_);
    take_published_ = true;
    start_time_ = std::chrono::steady_clock::now();

    TP_INFO(node_, "Waiting for TAKE completion on side %d (timeout: %d ms)", side_idx_, timeout_ms_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakePublisher::onRunning() {
    if (checkCondition()) {
        TP_INFO(node_, "TAKE confirmed on side %d", side_idx_);

        // Update blackboard: robot now holds hazelnuts and collection point becomes empty
        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::OCCUPIED;
        }

        // collection points are indexed 10-17 in GoalPose
        int collection_idx = target_pose_idx_ - PANTRY_LENGTH;
        if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info_.size())) {
            collection_info_[collection_idx] = FieldStatus::EMPTY;
        }

        blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status_);
        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info_);

        return BT::NodeStatus::SUCCESS;
    }

    // Check timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= timeout_ms_) {
        TP_WARN(node_, "TAKE on side %d timed out after %ld ms — proceeding anyway", side_idx_, elapsed);

        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::OCCUPIED;
        }

        int collection_idx = target_pose_idx_ - PANTRY_LENGTH;
        if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info_.size())) {
            collection_info_[collection_idx] = FieldStatus::EMPTY;
        }

        blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status_);
        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info_);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TakePublisher::onHalted() {
    TP_INFO(node_, "Halted (preempted)");
}

void TakePublisher::readBlackboard() {
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status_)) {
        TP_WARN(node_, "robot_side_status not in blackboard, using defaults");
        robot_side_status_ = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }

    if (!blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_info_)) {
        TP_WARN(node_, "collection_info not in blackboard, using defaults");
        collection_info_ = std::vector<FieldStatus>(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    }
}

void TakePublisher::publishTake(int side_idx) {
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        TP_WARN(node_, "Invalid side index: %d", side_idx);
        return;
    }

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(side_idx);
    TP_INFO(node_, "Publishing TAKE for side %d", side_idx);
    take_pub_->publish(msg);
}

bool TakePublisher::checkCondition() {
    std::vector<FieldStatus> current_status;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", current_status)) {
        return false;
    }

    if (side_idx_ < 0 || side_idx_ >= static_cast<int>(current_status.size())) {
        TP_WARN(node_, "Invalid side_idx %d, returning true to avoid blocking", side_idx_);
        return true;
    }

    return current_status[side_idx_] == FieldStatus::OCCUPIED;
}

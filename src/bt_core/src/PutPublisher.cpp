#include "PutPublisher.hpp"

// YELLOW colored logs for PutPublisher
#define PP_COLOR "\033[33m"  // Yellow
#define PP_RESET "\033[0m"
#define PP_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), PP_COLOR "[PutPublisher] " fmt PP_RESET, ##__VA_ARGS__)
#define PP_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), PP_COLOR "[PutPublisher] " fmt PP_RESET, ##__VA_ARGS__)
#define PP_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), PP_COLOR "[PutPublisher] " fmt PP_RESET, ##__VA_ARGS__)

PutPublisher::PutPublisher(const std::string& name, const BT::NodeConfig& config,
                           const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      side_idx_(0),
      target_pose_idx_(0),
      timeout_ms_(3000),
      put_published_(false) {
    put_pub_ = node_->create_publisher<std_msgs::msg::Int16>("/robot/on_put", 10);
    PP_INFO(node_, "Initialized — publishing to /robot/on_put");
}

BT::PortsList PutPublisher::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to use"),
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index (for pantry update)"),
        BT::InputPort<int>("timeout_ms", 3000, "Timeout in milliseconds")
    };
}

BT::NodeStatus PutPublisher::onStart() {
    // Read input ports
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        PP_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        PP_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        timeout_ms_ = 3000;
    }

    // Read blackboard
    readBlackboard();

    // Publish put command once
    publishPut(side_idx_);
    put_published_ = true;
    start_time_ = std::chrono::steady_clock::now();

    PP_INFO(node_, "Waiting for PUT completion on side %d (timeout: %d ms)", side_idx_, timeout_ms_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PutPublisher::onRunning() {
    if (checkCondition()) {
        PP_INFO(node_, "PUT confirmed on side %d", side_idx_);

        // Update blackboard: robot side becomes EMPTY and pantry becomes OCCUPIED
        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::EMPTY;
        }

        int pantry_idx = target_pose_idx_;
        if (pantry_idx >= 0 && pantry_idx < static_cast<int>(pantry_info_.size())) {
            pantry_info_[pantry_idx] = FieldStatus::OCCUPIED;
        }
        
        // Only write back robot_side_status and pantry_info for PUT
        // blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status_);
        blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_info_);

        return BT::NodeStatus::SUCCESS;
    }

    // Check timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= timeout_ms_) {
        PP_WARN(node_, "PUT on side %d timed out after %ld ms — proceeding anyway", side_idx_, elapsed);

        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::EMPTY;
        }

        int pantry_idx = target_pose_idx_;
        if (pantry_idx >= 0 && pantry_idx < static_cast<int>(pantry_info_.size())) {
            pantry_info_[pantry_idx] = FieldStatus::OCCUPIED;
        }

        // Only write back robot_side_status and pantry_info for PUT
        // blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status_);
        blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_info_);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void PutPublisher::onHalted() {
    PP_INFO(node_, "Halted (preempted)");
}

void PutPublisher::readBlackboard() {
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status_)) {
        PP_WARN(node_, "robot_side_status not in blackboard, using defaults");
        robot_side_status_ = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }

    if (!blackboard_->get<std::vector<FieldStatus>>("pantry_info", pantry_info_)) {
        PP_WARN(node_, "pantry_info not in blackboard, using defaults");
        pantry_info_ = std::vector<FieldStatus>(PANTRY_LENGTH, FieldStatus::EMPTY);
    }
}

void PutPublisher::publishPut(int side_idx) {
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        PP_WARN(node_, "Invalid side index: %d", side_idx);
        return;
    }

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(side_idx);
    PP_INFO(node_, "Publishing PUT for side %d", side_idx);
    put_pub_->publish(msg);
}

bool PutPublisher::checkCondition() {
    std::vector<FieldStatus> current_status;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", current_status)) {
        return false;
    }

    if (side_idx_ < 0 || side_idx_ >= static_cast<int>(current_status.size())) {
        PP_WARN(node_, "Invalid side_idx %d, returning true to avoid blocking", side_idx_);
        return true;
    }

    return current_status[side_idx_] == FieldStatus::EMPTY;
}

#include "TakePublisher_white.hpp"

// CYAN colored logs for TakePublisher_white
#define TPW_COLOR "\033[96m"  // Cyan
#define TPW_RESET "\033[0m"
#define TPW_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), TPW_COLOR "[TakePublisher_white] " fmt TPW_RESET, ##__VA_ARGS__)
#define TPW_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), TPW_COLOR "[TakePublisher_white] " fmt TPW_RESET, ##__VA_ARGS__)

TakePublisher_white::TakePublisher_white(const std::string& name, const BT::NodeConfig& config,
                                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      side_idx_(0),
      target_pose_idx_(0),
    timeout_ms_(3000) {
    take_pub_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/on_take", 10);

    hazelnut_status_ = std::vector<std::vector<FlipStatus>>(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));

        TPW_INFO(node_, "Initialized — publishing to /robot/on_take");
}

BT::PortsList TakePublisher_white::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to use"),
        BT::InputPort<int>("targetPoseIdx", 0, "Target pose index (for collection update)"),
        BT::InputPort<int>("timeout_ms", 3000, "Timeout in milliseconds")
    };
}

BT::NodeStatus TakePublisher_white::onStart() {
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        TPW_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    if (!getInput<int>("targetPoseIdx", target_pose_idx_)) {
        TPW_WARN(node_, "Missing targetPoseIdx, defaulting to 0");
        target_pose_idx_ = 0;
    }
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        timeout_ms_ = 3000;
    }

    readBlackboard();

    publishTakeArray(side_idx_);
    start_time_ = std::chrono::steady_clock::now();

    TPW_INFO(node_, "Waiting for TAKE completion on side %d (timeout: %d ms)", side_idx_, timeout_ms_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakePublisher_white::onRunning() {
    if (isTakeCompleted()) {
        TPW_INFO(node_, "TAKE confirmed on side %d", side_idx_);

        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::OCCUPIED;
        }

        int collection_idx = target_pose_idx_ - PANTRY_LENGTH;
        if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info_.size())) {
            collection_info_[collection_idx] = FieldStatus::EMPTY;
        }

        // 此輪 TAKE 完成後再清掉該 side 的暫存判斷，供下一輪重新判斷
        if (side_idx_ >= 0 && side_idx_ < ROBOT_SIDES) {
            for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
                hazelnut_status_[side_idx_][i] = FlipStatus::NO_FLIP;
            }
            blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_);
        }

        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info_);
        return BT::NodeStatus::SUCCESS;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();

    if (elapsed >= timeout_ms_) {
        TPW_WARN(node_, "TAKE on side %d timed out after %ld ms — proceeding anyway", side_idx_, elapsed);

        if (side_idx_ >= 0 && side_idx_ < static_cast<int>(robot_side_status_.size())) {
            robot_side_status_[side_idx_] = FieldStatus::OCCUPIED;
        }

        int collection_idx = target_pose_idx_ - PANTRY_LENGTH;
        if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info_.size())) {
            collection_info_[collection_idx] = FieldStatus::EMPTY;
        }

        // timeout 收斂時也清掉該 side，避免帶到下一輪
        if (side_idx_ >= 0 && side_idx_ < ROBOT_SIDES) {
            for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
                hazelnut_status_[side_idx_][i] = FlipStatus::NO_FLIP;
            }
            blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_);
        }

        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info_);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TakePublisher_white::onHalted() {
    TPW_INFO(node_, "Halted (preempted)");
}

void TakePublisher_white::readBlackboard() {
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_)) {
        TPW_WARN(node_, "hazelnut_status not in blackboard, using defaults");
    }
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status_)) {
        TPW_WARN(node_, "robot_side_status not in blackboard, using defaults");
        robot_side_status_ = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }
    if (!blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_info_)) {
        TPW_WARN(node_, "collection_info not in blackboard, using defaults");
        collection_info_ = std::vector<FieldStatus>(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    }
}

void TakePublisher_white::publishTakeArray(int side_idx) {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.resize(5);

    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        TPW_WARN(node_, "Invalid side index: %d", side_idx);
        return;
    }

    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        if (hazelnut_status_[side_idx][i] == FlipStatus::NEED_FLIP) {
            msg.data[i] = 1;
        } else if (hazelnut_status_[side_idx][i] == FlipStatus::NO_TAKE) {
            msg.data[i] = -1;
        } else {
            msg.data[i] = 0;
        }
    }

    msg.data[4] = static_cast<int16_t>(side_idx);

    TPW_INFO(node_, "Publishing TAKE array: [%d, %d, %d, %d, %d]",
             msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    take_pub_->publish(msg);
}

bool TakePublisher_white::isTakeCompleted() {
    std::vector<FieldStatus> current_status;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", current_status)) {
        return false;
    }

    if (side_idx_ < 0 || side_idx_ >= static_cast<int>(current_status.size())) {
        TPW_WARN(node_, "Invalid side_idx %d, returning true to avoid blocking", side_idx_);
        return true;
    }

    return current_status[side_idx_] == FieldStatus::OCCUPIED;
}

#include "MissionChecker.hpp"

// GREEN colored logs for MissionChecker
#define MC_COLOR "\033[32m"  // Green
#define MC_RESET "\033[0m"
#define MC_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), MC_COLOR "[MissionChecker] " fmt MC_RESET, ##__VA_ARGS__)
#define MC_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), MC_COLOR "[MissionChecker] " fmt MC_RESET, ##__VA_ARGS__)

MissionChecker::MissionChecker(const std::string& name, const BT::NodeConfig& config,
                               const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      timeout_ms_(3000),
      action_(ActionType::TAKE),
      side_idx_(0) {
    MC_INFO(node_, "Initialized");
}

BT::PortsList MissionChecker::providedPorts() {
    return {
        BT::InputPort<std::string>("ActionType"),
        BT::InputPort<int>("targetPoseSideIdx"),
        BT::InputPort<int>("timeout_ms", 3000, "Timeout in milliseconds")
    };
}

BT::NodeStatus MissionChecker::onStart() {
    // Read input ports
    std::string action_str;
    if (!getInput<std::string>("ActionType", action_str)) {
        MC_WARN(node_, "Missing ActionType, defaulting to 'take'");
        action_str = "take";
    }
    action_ = stringToActionType(action_str);
    
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        MC_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        MC_WARN(node_, "Missing timeout_ms, defaulting to 3000");
        timeout_ms_ = 3000;
    }
    
    start_time_ = std::chrono::steady_clock::now();
    
    MC_INFO(node_, "Waiting for %s completion on side %d (timeout: %d ms)",
            actionTypeToString(action_).c_str(), side_idx_, timeout_ms_);
    
    // Check immediately — maybe it's already done
    if (checkCondition()) {
        MC_INFO(node_, "%s confirmed immediately on side %d",
                actionTypeToString(action_).c_str(), side_idx_);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MissionChecker::onRunning() {
    // Check if condition is met
    if (checkCondition()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
        MC_INFO(node_, "%s confirmed by vision on side %d (took %ld ms)",
                actionTypeToString(action_).c_str(), side_idx_, elapsed);
        return BT::NodeStatus::SUCCESS;
    }
    
    // Check timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();
    
    if (elapsed >= timeout_ms_) {
        MC_WARN(node_, "%s on side %d timed out after %ld ms — proceeding anyway",
                actionTypeToString(action_).c_str(), side_idx_, elapsed);
        return BT::NodeStatus::SUCCESS;  // Don't block the game
    }
    
    return BT::NodeStatus::RUNNING;
}

void MissionChecker::onHalted() {
    MC_INFO(node_, "Halted (preempted)");
}

bool MissionChecker::checkCondition() {
    std::vector<FieldStatus> robot_side_status;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status)) {
        return false;  // Not available yet, keep waiting
    }
    
    if (side_idx_ < 0 || side_idx_ >= static_cast<int>(robot_side_status.size())) {
        MC_WARN(node_, "Invalid side_idx %d, returning true to avoid blocking", side_idx_);
        return true;
    }
    
    switch (action_) {
        case ActionType::TAKE:
            // TAKE is done when this side becomes OCCUPIED
            return robot_side_status[side_idx_] == FieldStatus::OCCUPIED;
        case ActionType::PUT:
            // PUT is done when this side becomes EMPTY
            return robot_side_status[side_idx_] == FieldStatus::EMPTY;
        case ActionType::FLIP:
            // FLIP has no vision feedback — always wait for full timeout
            return false;
        default:
            return true;
    }
}

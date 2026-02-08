#include "MissionPublisher.hpp"

// MAGENTA colored logs for MissionPublisher testing
#define MP_COLOR "\033[35m"  // Magenta
#define MP_RESET "\033[0m"
#define MP_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), MP_COLOR "[MissionPublisher] " fmt MP_RESET, ##__VA_ARGS__)
#define MP_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), MP_COLOR "[MissionPublisher] " fmt MP_RESET, ##__VA_ARGS__)
#define MP_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), MP_COLOR "[MissionPublisher] " fmt MP_RESET, ##__VA_ARGS__)

MissionPublisher::MissionPublisher(const std::string& name, const BT::NodeConfig& config,
                                   const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard) {
    
    // Create publishers for firmware commands
    flip_pub = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/on_flip", 10);
    take_pub = node_->create_publisher<std_msgs::msg::Int16>("/robot/on_take", 10);
    put_pub = node_->create_publisher<std_msgs::msg::Int16>("/robot/on_put", 10);
    
    // Initialize hazelnut status (4 sides x HAZELNUT_LENGTH slots)
    hazelnut_status = std::vector<std::vector<FlipStatus>>(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    
    RCLCPP_INFO(node_->get_logger(), "[MissionPublisher] Initialized publishers");
}

BT::PortsList MissionPublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("ActionType"),
        BT::InputPort<int>("targetPoseSideIdx"),
        BT::InputPort<int>("targetPoseIdx")  // For updating collection/pantry status
    };
}

BT::NodeStatus MissionPublisher::tick() {
    // Get action type from input port
    std::string action_str;
    if (!getInput<std::string>("ActionType", action_str)) {
        RCLCPP_ERROR(node_->get_logger(), "[MissionPublisher] Missing ActionType input");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get side index from input port
    int side_idx = 0;
    if (!getInput<int>("targetPoseSideIdx", side_idx)) {
        RCLCPP_WARN(node_->get_logger(), "[MissionPublisher] Missing targetPoseSideIdx, using default 0");
    }
    
    // Get hazelnut status from blackboard (for FLIP action)
    if (blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] Got hazelnut_status from blackboard");
    }
    
    // Get target pose index for collection/pantry status updates
    int target_pose_idx = 0;
    getInput<int>("targetPoseIdx", target_pose_idx);
    
    // Convert action string to ActionType
    ActionType action = stringToActionType(action_str);
    
    RCLCPP_INFO(node_->get_logger(), "[MissionPublisher] tick: action=%s, side=%d", 
                action_str.c_str(), side_idx);
    
    // Dispatch to appropriate handler
    switch (action) {
        case ActionType::FLIP:
            publishFlip(side_idx);
            break;
        case ActionType::TAKE:
            publishTake(side_idx, target_pose_idx);
            break;
        case ActionType::PUT:
            publishPut(side_idx, target_pose_idx);
            break;
        default:
            RCLCPP_WARN(node_->get_logger(), "[MissionPublisher] Unknown action: %s", action_str.c_str());
            return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Publish flip command to firmware
 * 
 * Topic: /robot/on_flip
 * Type: Int16MultiArray (length 5)
 *   - index 0-3: which hazelnut slots need flipping (1=flip, 0=no flip)
 *   - index 4: which side to perform the flip action
 * 
 * Example: [1, 0, 1, 1, 3] means flip slots 0, 2, 3 on side 3
 * 
 * After publishing, resets hazelnut_status for this side to NO_FLIP
 */
void MissionPublisher::publishFlip(int side_idx) {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.resize(5);
    
    // Check if side_idx is valid
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        RCLCPP_ERROR(node_->get_logger(), "[MissionPublisher] Invalid side index: %d", side_idx);
        return;
    }
    
    // Fill in which hazelnuts need flipping for this side
    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        if (hazelnut_status[side_idx][i] == FlipStatus::NEED_FLIP) {
            msg.data[i] = 1;  // Need to flip
        } else {
            msg.data[i] = 0;  // No flip needed
        }
    }
    
    // Set side index in last position
    msg.data[4] = static_cast<int16_t>(side_idx);
    
    RCLCPP_INFO(node_->get_logger(), 
                "[MissionPublisher] Publishing FLIP: [%d, %d, %d, %d, %d]",
                msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    
    flip_pub->publish(msg);
    
    // Reset hazelnut_status to NO_FLIP for this side (trust firmware will flip them)
    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        hazelnut_status[side_idx][i] = FlipStatus::NO_FLIP;
    }
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
    
    RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] Reset hazelnut_status for side %d to NO_FLIP", side_idx);
}

/**
 * @brief Publish take command to firmware
 * 
 * Topic: /robot/on_take
 * Type: Int16
 *   - data: which side to perform the take action
 * 
 * After TAKE:
 *   - robot_side_status[side_idx] = OCCUPIED (robot now holds hazelnuts)
 *   - collection_info[collection_idx] = EMPTY (collection point is now empty)
 */
void MissionPublisher::publishTake(int side_idx, int target_pose_idx) {
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(side_idx);
    
    RCLCPP_INFO(node_->get_logger(), "[MissionPublisher] Publishing TAKE: side=%d, pose=%d", 
                side_idx, target_pose_idx);
    
    take_pub->publish(msg);
    
    // Update robot_side_status: this side now holds hazelnuts
    std::vector<FieldStatus> robot_side_status;
    if (blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status)) {
        if (side_idx >= 0 && side_idx < static_cast<int>(robot_side_status.size())) {
            robot_side_status[side_idx] = FieldStatus::OCCUPIED;
            blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
            RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] robot_side_status[%d] = OCCUPIED", side_idx);
        }
    }
    
    // Update collection_info: this collection point is now empty
    // Collection points start at PANTRY_LENGTH in GoalPose enum
    int collection_idx = target_pose_idx - PANTRY_LENGTH;
    std::vector<FieldStatus> collection_info;
    if (blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_info)) {
        if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info.size())) {
            collection_info[collection_idx] = FieldStatus::EMPTY;
            blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info);
            RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] collection_info[%d] = EMPTY", collection_idx);
        }
    }
}

/**
 * @brief Publish put command to firmware
 * 
 * Topic: /robot/on_put
 * Type: Int16
 *   - data: which side to perform the put action
 * 
 * After PUT:
 *   - robot_side_status[side_idx] = EMPTY (robot released hazelnuts)
 *   - pantry_info[pantry_idx] = OCCUPIED (pantry now has hazelnuts)
 */
void MissionPublisher::publishPut(int side_idx, int target_pose_idx) {
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(side_idx);
    
    RCLCPP_INFO(node_->get_logger(), "[MissionPublisher] Publishing PUT: side=%d, pose=%d", 
                side_idx, target_pose_idx);
    
    put_pub->publish(msg);
    
    // Update robot_side_status: this side is now empty
    std::vector<FieldStatus> robot_side_status;
    if (blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status)) {
        if (side_idx >= 0 && side_idx < static_cast<int>(robot_side_status.size())) {
            robot_side_status[side_idx] = FieldStatus::EMPTY;
            blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
            RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] robot_side_status[%d] = EMPTY", side_idx);
        }
    }
    
    // Update pantry_info: this pantry is now occupied
    int pantry_idx = target_pose_idx;  // Pantry points are 0 to PANTRY_LENGTH-1
    std::vector<FieldStatus> pantry_info;
    if (blackboard_->get<std::vector<FieldStatus>>("pantry_info", pantry_info)) {
        if (pantry_idx >= 0 && pantry_idx < static_cast<int>(pantry_info.size())) {
            pantry_info[pantry_idx] = FieldStatus::OCCUPIED;
            blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_info);
            RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] pantry_info[%d] = OCCUPIED", pantry_idx);
        }
    }
}

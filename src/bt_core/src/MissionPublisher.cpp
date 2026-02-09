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
    
    // Initialize state vectors
    robot_side_status = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    collection_info = std::vector<FieldStatus>(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    pantry_info = std::vector<FieldStatus>(PANTRY_LENGTH, FieldStatus::EMPTY);
    hazelnut_status = std::vector<std::vector<FlipStatus>>(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    
    MP_INFO(node_, "Initialized publishers");
}

void MissionPublisher::readBlackboard() {
    // Read current state from blackboard into member variables
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_side_status)) {
        MP_INFO(node_, "robot_side_status not in blackboard, using defaults");
    }
    
    if (!blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_info)) {
        MP_INFO(node_, "collection_info not in blackboard, using defaults");
    }
    
    if (!blackboard_->get<std::vector<FieldStatus>>("pantry_info", pantry_info)) {
        MP_INFO(node_, "pantry_info not in blackboard, using defaults");
    }
    
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        MP_INFO(node_, "hazelnut_status not in blackboard, using defaults");
    }
    // print hazelnut_status 
    for (int side = 0; side < ROBOT_SIDES; ++side) {
        std::string status_str = "Side " + std::to_string(side) + ": [";
        for (int slot = 0; slot < HAZELNUT_LENGTH; ++slot) {
            status_str += std::to_string(static_cast<int>(hazelnut_status[side][slot]));
            if (slot < HAZELNUT_LENGTH - 1) status_str += ", ";
        }
        status_str += "]";
        MP_INFO(node_, "%s", status_str.c_str());
    }
}

void MissionPublisher::writeBlackboard() {
    // Write updated state back to blackboard
    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
    blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info);
    blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_info);
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
}

BT::PortsList MissionPublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("ActionType"),
        BT::InputPort<int>("targetPoseSideIdx"),
        BT::InputPort<int>("targetPoseIdx")  // For updating collection/pantry status
    };
}

BT::NodeStatus MissionPublisher::tick() {
    // Read current state from blackboard
    readBlackboard();
    
    // Get action type from input port
    std::string action_str;
    if (!getInput<std::string>("ActionType", action_str)) {
        MP_ERROR(node_, "Missing ActionType input");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get side index from input port
    int side_idx = 0;
    if (!getInput<int>("targetPoseSideIdx", side_idx)) {
        MP_WARN(node_, "Missing targetPoseSideIdx, using default 0");
    }
    
    // Get target pose index for collection/pantry status updates
    int target_pose_idx = 0;
    getInput<int>("targetPoseIdx", target_pose_idx);
    
    // Convert action string to ActionType
    ActionType action = stringToActionType(action_str);
    
    MP_INFO(node_, "tick: action=%s, side=%d, pose=%d", 
            action_str.c_str(), side_idx, target_pose_idx);
    
    // Dispatch to appropriate handler
    switch (action) {
        case ActionType::FLIP:
            publishFlip(side_idx);
            // Only write back robot_side_status and hazelnut_status for FLIP
            blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
            blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
            break;
        case ActionType::TAKE:
            publishTake(side_idx, target_pose_idx);
            // Only write back robot_side_status and collection_info for TAKE
            blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
            blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_info);
            break;
        case ActionType::PUT:
            publishPut(side_idx, target_pose_idx);
            // Only write back robot_side_status and pantry_info for PUT
            blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_side_status);
            blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_info);
            break;
        default:
            MP_WARN(node_, "Unknown action: %s", action_str.c_str());
            return BT::NodeStatus::FAILURE;
    }
    
    // Don't call writeBlackboard() here - we already wrote only what we modified above
    
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
    
    MP_INFO(node_, 
            "Publishing FLIP: [%d, %d, %d, %d, %d]",
            msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    flip_pub->publish(msg);
    
    // Reset hazelnut_status to NO_FLIP for this side (trust firmware will flip them)
    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        hazelnut_status[side_idx][i] = FlipStatus::NO_FLIP;
    }
    
    MP_INFO(node_, "Reset hazelnut_status for side %d to NO_FLIP", side_idx);
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
    
    MP_INFO(node_, "TAKE from collection %d (robot side %d now OCCUPIED)", 
            target_pose_idx, side_idx);
    
    take_pub->publish(msg);
    
    // Update robot_side_status: this side now holds hazelnuts
    if (side_idx >= 0 && side_idx < static_cast<int>(robot_side_status.size())) {
        robot_side_status[side_idx] = FieldStatus::OCCUPIED;
        RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] robot_side_status[%d] = OCCUPIED", side_idx);
    }
    
    // Update collection_info: this collection point is now empty
    // Collection points start at PANTRY_LENGTH in GoalPose enum
    // But collection_info vector is 0-indexed for collections
    // GoalPose: 0-9 (Pantry), 10-19 (Collection)
    // So if target_pose_idx is 10 (Collection 0), collection_idx should be 0.
    int collection_idx = target_pose_idx - 10; // PANTRY_LENGTH is 10
    
    if (collection_idx >= 0 && collection_idx < static_cast<int>(collection_info.size())) {
        collection_info[collection_idx] = FieldStatus::EMPTY;
        MP_INFO(node_, "collection_info[%d] (GoalPose %d) = EMPTY", 
                    collection_idx, target_pose_idx);
    } else {
        MP_INFO(node_, "Invalid collection_idx %d for size %zu", 
                    collection_idx, collection_info.size());
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
    
    MP_INFO(node_, "PUT to pantry %d (robot side %d now EMPTY)", target_pose_idx, side_idx);
    
    put_pub->publish(msg);
    
    // Update robot_side_status: this side is now empty
    if (side_idx >= 0 && side_idx < static_cast<int>(robot_side_status.size())) {
        robot_side_status[side_idx] = FieldStatus::EMPTY;
        RCLCPP_DEBUG(node_->get_logger(), "[MissionPublisher] robot_side_status[%d] = EMPTY", side_idx);
    }
    
    // Update pantry_info: this pantry is now occupied
    // Pantry points are 0-9 in GoalPose enum
    int pantry_idx = target_pose_idx; // 0-indexed for pantry
    
    if (pantry_idx >= 0 && pantry_idx < static_cast<int>(pantry_info.size())) {
        pantry_info[pantry_idx] = FieldStatus::OCCUPIED;
        MP_INFO(node_, "pantry_info[%d] = OCCUPIED", pantry_idx);
    } else {
        MP_INFO(node_, "Invalid pantry_idx %d for size %zu", 
                    pantry_idx, pantry_info.size());
    }
}

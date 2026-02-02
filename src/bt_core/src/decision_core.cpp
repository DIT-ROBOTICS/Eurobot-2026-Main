#include "decision_core.hpp"

// ============ Constructor ============

DecisionCore::DecisionCore(const std::string& name, const BT::NodeConfig& config, 
                           const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), 
      node_(params.nh.lock()), 
      blackboard_(blackboard),
      current_sequence_index_(0) {
    
    loadMapPoints();
}

// ============ Static Methods ============

BT::PortsList DecisionCore::providedPorts() {
    return {
        BT::OutputPort<int>("next_target_index"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("next_target_pose"),
        BT::OutputPort<std::string>("action_type")
    };
}

// ============ Main Tick ============

BT::NodeStatus DecisionCore::tick() {
    RCLCPP_INFO(node_->get_logger(), "[DecisionCore] Evaluating next action...");

    // Get mission sequence
    std::vector<int> sequence = getMissionSequence();

    // Get sensor data for future decision-making
    std_msgs::msg::Int32MultiArray collection_info, pantry_info;
    getSensorData(collection_info, pantry_info);

    // Check if sequence complete
    if (isSequenceComplete(sequence.size())) {
        RCLCPP_INFO(node_->get_logger(), "[DecisionCore] All sequence points completed");
        resetSequence();
        return BT::NodeStatus::SUCCESS;
    }

    // Get current target
    int target_index = sequence[current_sequence_index_];
    RCLCPP_INFO(node_->get_logger(), "[DecisionCore] Sequence[%d] -> Point %d", 
                current_sequence_index_, target_index);

    // Convert to pose
    geometry_msgs::msg::PoseStamped target_pose = indexToPose(target_index);
    if (target_pose.header.frame_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[DecisionCore] Invalid target_index %d", target_index);
        return BT::NodeStatus::FAILURE;
    }

    // Determine action type
    std::string action_type = determineActionType(target_index);

    // Write outputs
    writeOutputs(target_index, target_pose, action_type);

    // Advance sequence for next tick
    advanceSequence();

    return BT::NodeStatus::SUCCESS;
}

// ============ Private Helper Methods ============

void DecisionCore::loadMapPoints() {
    // Declare parameter if not already declared
    if (!node_->has_parameter("map_points")) {
        node_->declare_parameter("map_points", std::vector<double>{});
    }

    // Get parameter
    if (node_->get_parameter("map_points", map_points_)) {
        RCLCPP_INFO(node_->get_logger(), "[DecisionCore] Loaded %zu map point values", 
                    map_points_.size());
    } else {
        RCLCPP_WARN(node_->get_logger(), "[DecisionCore] Failed to load map_points param");
    }
}

std::vector<int> DecisionCore::getMissionSequence() {
    std::vector<int> sequence;
    if (blackboard_->get<std::vector<int>>("json_point", sequence)) {
        RCLCPP_DEBUG(node_->get_logger(), "[DecisionCore] Found json_point with %zu entries", 
                     sequence.size());
    } else {
        RCLCPP_WARN(node_->get_logger(), "[DecisionCore] No json_point, using default sequence");
        sequence = DEFAULT_SEQUENCE;
    }
    return sequence;
}

void DecisionCore::getSensorData(std_msgs::msg::Int32MultiArray& collection_info,
                                  std_msgs::msg::Int32MultiArray& pantry_info) {
    if (!blackboard_->get<std_msgs::msg::Int32MultiArray>("collection_info", collection_info)) {
        RCLCPP_DEBUG(node_->get_logger(), "[DecisionCore] Failed to get collection_info from blackboard");
    }
    if (!blackboard_->get<std_msgs::msg::Int32MultiArray>("pantry_info", pantry_info)) {
        RCLCPP_DEBUG(node_->get_logger(), "[DecisionCore] Failed to get pantry_info from blackboard");
    }
    
    // Log sensor data for debugging
    if (!collection_info.data.empty()) {
        RCLCPP_DEBUG(node_->get_logger(), "[DecisionCore] Collection data size: %zu", 
                     collection_info.data.size());
    }
}

geometry_msgs::msg::PoseStamped DecisionCore::indexToPose(int point_index) {
    geometry_msgs::msg::PoseStamped pose;
    
    int data_index = point_index * VALUES_PER_POINT;
    
    // Validate index
    if (map_points_.empty() || data_index + 2 >= static_cast<int>(map_points_.size())) {
        RCLCPP_ERROR(node_->get_logger(), "[DecisionCore] Point index %d out of range", point_index);
        return pose;  // Return empty pose (frame_id will be empty)
    }
    
    // Set header
    pose.header.frame_id = "map";
    pose.header.stamp = node_->get_clock()->now();
    
    // Set position
    pose.pose.position.x = map_points_[data_index];
    pose.pose.position.y = map_points_[data_index + 1];
    pose.pose.position.z = 0.0;
    
    // Set orientation from direction
    double direction = map_points_[data_index + 2];
    pose.pose.orientation = directionToQuaternion(direction);
    
    RCLCPP_INFO(node_->get_logger(), "[DecisionCore] Target: (%.2f, %.2f) dir=%.0f",
                pose.pose.position.x, pose.pose.position.y, direction);
    
    return pose;
}

geometry_msgs::msg::Quaternion DecisionCore::directionToQuaternion(double direction) {
    // Direction encoding: 0=East (+X), 1=North (+Y), 2=West (-X), 3=South (-Y)
    double yaw = direction * M_PI / 2.0;
    
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
    
    return q;
}

std::string DecisionCore::determineActionType([[maybe_unused]] int point_index) {
    // TODO: Implement smarter action selection based on:
    // - Point type (collection vs pantry vs other)
    // - Current robot state
    // - Sensor data
    
    // For now, default to navigate
    return "navigate";
}

bool DecisionCore::isSequenceComplete(size_t sequence_size) {
    return current_sequence_index_ >= static_cast<int>(sequence_size);
}

void DecisionCore::resetSequence() {
    current_sequence_index_ = 0;
}

void DecisionCore::advanceSequence() {
    current_sequence_index_++;
}

void DecisionCore::writeOutputs(int target_index, 
                                 const geometry_msgs::msg::PoseStamped& target_pose,
                                 const std::string& action_type) {
    // Set BT output ports
    setOutput("next_target_index", target_index);
    setOutput("next_target_pose", target_pose);
    setOutput("action_type", action_type);

    // Also write to blackboard for other nodes to access
    blackboard_->set<int>("current_target_index", target_index);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("current_target_pose", target_pose);
    blackboard_->set<std::string>("current_action_type", action_type);
}

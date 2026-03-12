#include "FlipPublisher.hpp"

// BRIGHT GREEN colored logs for FlipPublisher
#define FP_COLOR "\033[92m"  // Bright Green
#define FP_RESET "\033[0m"
#define FP_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), FP_COLOR "[FlipPublisher] " fmt FP_RESET, ##__VA_ARGS__)
#define FP_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), FP_COLOR "[FlipPublisher] " fmt FP_RESET, ##__VA_ARGS__)

FlipPublisher::FlipPublisher(const std::string& name, const BT::NodeConfig& config,
                             const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      side_idx_(0),
      timeout_ms_(5000),
      is_docking_received_(false),
      flip_published_(false) {
    
    // Create publisher for flip commands
    flip_pub_ = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/on_flip", 10);
    
    // Subscribe to isDocking signal
    is_docking_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/robot/isDocking", 10,
        std::bind(&FlipPublisher::isDockingCallback, this, std::placeholders::_1));
    
    // Initialize hazelnut status
    hazelnut_status_ = std::vector<std::vector<FlipStatus>>(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    
    FP_INFO(node_, "Initialized — subscribing to /robot/isDocking, publishing to /robot/on_flip");
}

BT::PortsList FlipPublisher::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to flip"),
        BT::InputPort<int>("timeout_ms", 5000, "Timeout in milliseconds")
    };
}

BT::NodeStatus FlipPublisher::onStart() {
    // Read input ports
    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        FP_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }
    if (!getInput<int>("timeout_ms", timeout_ms_)) {
        timeout_ms_ = 5000;
    }
    
    // Read hazelnut status from blackboard
    readBlackboard();
    
    // Reset state
    is_docking_received_ = false;
    flip_published_ = false;
    start_time_ = std::chrono::steady_clock::now();
    
    FP_INFO(node_, "Waiting for isDocking signal to flip side %d (timeout: %d ms)", side_idx_, timeout_ms_);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FlipPublisher::onRunning() {
    // If isDocking received (data == 1) and we haven't published yet, publish flip
    if (is_docking_received_ && !flip_published_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();
        FP_INFO(node_, "isDocking received! Publishing flip for side %d (waited %ld ms)", side_idx_, elapsed);
        publishFlip(side_idx_);
        flip_published_ = true;
        
        // Update blackboard: reset hazelnut_status for this side
        blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_);
        
        return BT::NodeStatus::SUCCESS;
    }
    
    // Check timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();
    
    if (elapsed >= timeout_ms_) {
        FP_WARN(node_, "Timeout (%d ms) — publishing flip anyway for side %d", timeout_ms_, side_idx_);
        publishFlip(side_idx_);
        flip_published_ = true;
        blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

void FlipPublisher::onHalted() {
    FP_INFO(node_, "Halted (preempted)");
}

void FlipPublisher::isDockingCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    if (msg->data == 1) {
        is_docking_received_ = true;
    }
}

void FlipPublisher::readBlackboard() {
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status_)) {
        FP_WARN(node_, "hazelnut_status not in blackboard, using defaults");
    }
    
    // Print hazelnut status for debugging
    for (int side = 0; side < ROBOT_SIDES; ++side) {
        std::string status_str = "Side " + std::to_string(side) + ": [";
        for (int slot = 0; slot < HAZELNUT_LENGTH; ++slot) {
            status_str += std::to_string(static_cast<int>(hazelnut_status_[side][slot]));
            if (slot < HAZELNUT_LENGTH - 1) status_str += ", ";
        }
        status_str += "]";
        FP_INFO(node_, "%s", status_str.c_str());
    }
}

/**
 * @brief Publish flip command to firmware
 * 
 * Same logic as MissionPublisher::publishFlip:
 *   Topic: /robot/on_flip
 *   Type: Int16MultiArray (length 5)
 *     - index 0-3: which hazelnut slots need flipping (1=flip, 0=no flip)
 *     - index 4: which side to perform the flip action
 * 
 * After publishing, resets hazelnut_status for this side to NO_FLIP
 */
void FlipPublisher::publishFlip(int side_idx) {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.resize(5);
    
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        FP_WARN(node_, "Invalid side index: %d", side_idx);
        return;
    }
    
    // Fill in which hazelnuts need flipping for this side
    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        msg.data[i] = (hazelnut_status_[side_idx][i] == FlipStatus::NEED_FLIP) ? 1 : 0;
    }
    
    // Set side index in last position
    msg.data[4] = static_cast<int16_t>(side_idx);
    
    FP_INFO(node_, "Publishing FLIP: [%d, %d, %d, %d, %d]",
            msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
    flip_pub_->publish(msg);
    
    // Reset hazelnut_status to NO_FLIP for this side
    for (int i = 0; i < HAZELNUT_LENGTH; ++i) {
        hazelnut_status_[side_idx][i] = FlipStatus::NO_FLIP;
    }
    
    FP_INFO(node_, "Reset hazelnut_status for side %d to NO_FLIP", side_idx);
}

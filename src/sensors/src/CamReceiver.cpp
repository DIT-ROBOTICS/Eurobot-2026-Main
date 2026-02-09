#include "CamReceiver.hpp"
#include "bt_config.hpp"

CamReceiver::CamReceiver(const std::string& name, const BT::NodeConfig& config, 
                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), 
      node_(params.nh.lock()), 
      blackboard_(blackboard),
      running_(true) {

    // Create a dedicated callback group for our subscriptions
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Create a dedicated executor for spinning our callback group
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

    // Create subscription options with our callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // Subscribe to collection status
    collection_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "collection_status", 10, 
        std::bind(&CamReceiver::collection_info_callback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to pantry status
    pantry_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "pantry_status", 10, 
        std::bind(&CamReceiver::pantry_info_callback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to hazelnut flip information
    hazelnut_flip_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/robot/vision/hazelnut/flip", 10,
        std::bind(&CamReceiver::hazelnut_flip_callback, this, std::placeholders::_1),
        sub_options);
    
    // Initialize with default values (camera fallback)
    initializeDefaultStatus();
    
    // Start background spin thread
    spin_thread_ = std::thread(&CamReceiver::spinThread, this);
    
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Started with background spin thread");
}

CamReceiver::~CamReceiver() {
    // Stop the spin thread
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Destroyed, spin thread stopped");
}

void CamReceiver::spinThread() {
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Spin thread started");
    while (running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Spin thread exiting");
}

void CamReceiver::initializeDefaultStatus() {
    // Initialize pantry_info: all EMPTY (available to put hazelnuts)
    std::vector<FieldStatus> default_pantry(PANTRY_LENGTH, FieldStatus::EMPTY);
    blackboard_->set<std::vector<FieldStatus>>("pantry_info", default_pantry);
    
    // Initialize collection_info: all OCCUPIED (has hazelnuts to take)
    std::vector<FieldStatus> default_collection(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    blackboard_->set<std::vector<FieldStatus>>("collection_info", default_collection);
    
    // Initialize robot_side_status: all EMPTY (robot sides have no hazelnuts)
    std::vector<FieldStatus> default_robot_sides(ROBOT_SIDES, FieldStatus::EMPTY);
    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", default_robot_sides);
    
    // Initialize hazelnut_status: all NO_FLIP
    std::vector<std::vector<FlipStatus>> default_hazelnut(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", default_hazelnut);
}

PortsList CamReceiver::providedPorts() {
    return {};
}

void CamReceiver::collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received collection info update.");
    collection_info = *msg;
    
    if (!collection_info.data.empty()) {
        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> collection_status;
        for (const auto& val : collection_info.data) {
            collection_status.push_back(static_cast<FieldStatus>(val));
        }
        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("collection_info_raw", collection_info);
    }
}

void CamReceiver::pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received pantry info update.");
    pantry_info = *msg;
    
    if (!pantry_info.data.empty()) {
        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> pantry_status;
        for (const auto& val : pantry_info.data) {
            pantry_status.push_back(static_cast<FieldStatus>(val));
        }
        blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("pantry_info_raw", pantry_info);
    }
}

void CamReceiver::hazelnut_flip_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received hazelnut flip info update.");
    
    if (msg->data.size() < 5) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid hazelnut flip data size: %zu", msg->data.size());
        return;
    }
    
    // Parse message: indices 0-3 are flip status (0=NO_FLIP, 1=NEED_FLIP), index 4 is side index
    int side_idx = msg->data[4];
    
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid side index: %d", side_idx);
        return;
    }
    
    // Get current hazelnut_status from blackboard
    std::vector<std::vector<FlipStatus>> hazelnut_status;
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        // Initialize if not found
        hazelnut_status = std::vector<std::vector<FlipStatus>>(
            ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    }
    
    // Update the specified side with flip information
    for (int i = 0; i < std::min(4, HAZELNUT_LENGTH); ++i) {
        if (msg->data[i] == 1) {
            hazelnut_status[side_idx][i] = FlipStatus::NEED_FLIP;
        } else {
            hazelnut_status[side_idx][i] = FlipStatus::NO_FLIP;
        }
    }
    
    // Write back to blackboard
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
    
    RCLCPP_INFO(node_->get_logger(), 
                "CamReceiver: Updated hazelnut flip status for side %d: [%d, %d, %d, %d]",
                side_idx, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

BT::NodeStatus CamReceiver::tick() {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver tick");
    
    // Log current status for debugging
    std::vector<FieldStatus> pantry_status;
    std::vector<FieldStatus> collection_status;
    
    if (blackboard_->get<std::vector<FieldStatus>>("pantry_info", pantry_status)) {
        for (size_t i = 0; i < pantry_status.size(); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "Pantry[%zu]: %d", i, static_cast<int>(pantry_status[i]));
        }
    }
    
    if (blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_status)) {
        for (size_t i = 0; i < collection_status.size(); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "Collection[%zu]: %d", i, static_cast<int>(collection_status[i]));
        }
    }
    
    return BT::NodeStatus::SUCCESS;
}
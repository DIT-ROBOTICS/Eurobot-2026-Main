#include "FirmwareReceiver.hpp"
#include "bt_config.hpp"

// BRIGHT CYAN colored logs for FirmwareReceiver
#define FR_COLOR "\033[96m"  // Bright Cyan
#define FR_RESET "\033[0m"
#define FR_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), FR_COLOR "[FirmwareReceiver] " fmt FR_RESET, ##__VA_ARGS__)
#define FR_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), FR_COLOR "[FirmwareReceiver] " fmt FR_RESET, ##__VA_ARGS__)
#define FR_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), FR_COLOR "[FirmwareReceiver] " fmt FR_RESET, ##__VA_ARGS__)

FirmwareReceiver::FirmwareReceiver(const std::string& name, const BT::NodeConfig& config, 
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

    // Subscribe to flip finish
    flip_finish_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/robot/firmware/finish/flip", 10, 
        std::bind(&FirmwareReceiver::flipFinishCallback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to put finish
    put_finish_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/robot/firmware/finish/put", 10, 
        std::bind(&FirmwareReceiver::putFinishCallback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to take finish
    take_finish_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/robot/firmware/finish/take", 10, 
        std::bind(&FirmwareReceiver::takeFinishCallback, this, std::placeholders::_1),
        sub_options);
    
    // Start background spin thread
    spin_thread_ = std::thread(&FirmwareReceiver::spinThread, this);
    
    FR_INFO(node_, "Started with background spin thread");
}

FirmwareReceiver::~FirmwareReceiver() {
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    FR_INFO(node_, "Destroyed, spin thread stopped");
}

void FirmwareReceiver::spinThread() {
    FR_INFO(node_, "Spin thread started");
    while (running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FR_INFO(node_, "Spin thread exiting");
}

PortsList FirmwareReceiver::providedPorts() {
    return {};
}

void FirmwareReceiver::flipFinishCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    int side_idx = msg->data;

    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        FR_WARN(node_, "Flip finish - invalid side index: %d", side_idx);
        return;
    }

    // Flip finish: do nothing, just log
    FR_INFO(node_, "Flip finish received for side %d (no action taken)", side_idx);
}

void FirmwareReceiver::putFinishCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    int side_idx = msg->data;

    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        FR_WARN(node_, "Put finish - invalid side index: %d", side_idx);
        return;
    }

    // Put finish: set robot_side_status[side_idx] to EMPTY
    std::vector<FieldStatus> robot_sides;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_sides)) {
        robot_sides = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }

    robot_sides[side_idx] = FieldStatus::EMPTY;
    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_sides);

    FR_INFO(node_, "Put finish - set robot_side_status[%d] to EMPTY", side_idx);
}

void FirmwareReceiver::takeFinishCallback(const std_msgs::msg::Int16::SharedPtr msg) {
    int side_idx = msg->data;

    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        FR_WARN(node_, "Take finish - invalid side index: %d", side_idx);
        return;
    }

    // Take finish: set robot_side_status[side_idx] to OCCUPIED
    std::vector<FieldStatus> robot_sides;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_sides)) {
        robot_sides = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }

    robot_sides[side_idx] = FieldStatus::OCCUPIED;
    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_sides);

    FR_INFO(node_, "Take finish - set robot_side_status[%d] to OCCUPIED", side_idx);
}

BT::NodeStatus FirmwareReceiver::tick() {
    RCLCPP_DEBUG(node_->get_logger(), "[FirmwareReceiver]: tick");
    return BT::NodeStatus::SUCCESS;
}

#include "GameInfoReceiver.hpp"

GameInfoReceiver::GameInfoReceiver(const std::string& name, const BT::NodeConfig& config, 
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

    // Subscribe to game time
    game_time_sub = node_->create_subscription<std_msgs::msg::Float32>(
        "/robot/startup/game_time", 2, 
        std::bind(&GameInfoReceiver::game_time_callback, this, std::placeholders::_1),
        sub_options);
    
    // Initialize game time
    game_time = 0.0;
    
    // Start background spin thread
    spin_thread_ = std::thread(&GameInfoReceiver::spinThread, this);
    
    RCLCPP_INFO(node_->get_logger(), "GameInfoReceiver: Started with background spin thread");
}

GameInfoReceiver::~GameInfoReceiver() {
    // Stop the spin thread
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    RCLCPP_INFO(node_->get_logger(), "GameInfoReceiver: Destroyed, spin thread stopped");
}

void GameInfoReceiver::spinThread() {
    RCLCPP_INFO(node_->get_logger(), "GameInfoReceiver: Spin thread started");
    while (running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_->get_logger(), "GameInfoReceiver: Spin thread exiting");
}

BT::PortsList GameInfoReceiver::providedPorts() {
    return {};
}

void GameInfoReceiver::game_time_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
    
    // Write to blackboard
    blackboard_->set<float>("game_time", game_time);
    
    RCLCPP_DEBUG(node_->get_logger(), "GameInfoReceiver: Game time updated to %.2f", game_time);
}

BT::NodeStatus GameInfoReceiver::tick() {
    RCLCPP_DEBUG(node_->get_logger(), "GameInfoReceiver: tick");
    
    // Log current game time (updated by background thread)
    RCLCPP_DEBUG(node_->get_logger(), "Game time: %.2f", game_time);
    
    return BT::NodeStatus::SUCCESS;
}

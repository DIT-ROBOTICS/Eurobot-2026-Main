#include "LocReceiver.hpp"

LocReceiver::LocReceiver(const std::string& name, const BT::NodeConfig& config, 
                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), 
      node_(params.nh.lock()), 
      blackboard_(blackboard),
      running_(true),
      robot_pose_received(false),
      rival_pose_received(false) {

    // Create a dedicated callback group for our subscriptions
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Create a dedicated executor for spinning our callback group
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

    // Create subscription options with our callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // Subscribe to robot pose from localization
    robot_pose_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/final_pose", 10, 
        std::bind(&LocReceiver::robot_pose_callback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to rival pose from tracking
    rival_pose_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/rhino_pose", 10, 
        std::bind(&LocReceiver::rival_pose_callback, this, std::placeholders::_1),
        sub_options);
    
    // Initialize poses
    robot_pose.header.frame_id = "map";
    rival_pose.header.frame_id = "map";
    
    // Start background spin thread
    spin_thread_ = std::thread(&LocReceiver::spinThread, this);
    
    RCLCPP_INFO(node_->get_logger(), "LocReceiver: Started with background spin thread");
}

LocReceiver::~LocReceiver() {
    // Stop the spin thread
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    RCLCPP_INFO(node_->get_logger(), "LocReceiver: Destroyed, spin thread stopped");
}

void LocReceiver::spinThread() {
    RCLCPP_INFO(node_->get_logger(), "LocReceiver: Spin thread started");
    while (running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_->get_logger(), "LocReceiver: Spin thread exiting");
}

BT::PortsList LocReceiver::providedPorts() {
    return {};
}

void LocReceiver::robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert Odometry to PoseStamped
    robot_pose.header = msg->header;
    robot_pose.pose = msg->pose.pose;
    robot_pose_received = true;
    
    // Write to blackboard
    blackboard_->set<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose);
    
    RCLCPP_DEBUG(node_->get_logger(), "LocReceiver: Robot pose updated (%.2f, %.2f)", 
                 robot_pose.pose.position.x, robot_pose.pose.position.y);
}

void LocReceiver::rival_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert Odometry to PoseStamped
    rival_pose.header = msg->header;
    rival_pose.pose = msg->pose.pose;
    rival_pose_received = true;
    
    // Write to blackboard
    blackboard_->set<geometry_msgs::msg::PoseStamped>("rival_pose", rival_pose);
    
    RCLCPP_DEBUG(node_->get_logger(), "LocReceiver: Rival pose updated (%.2f, %.2f)", 
                 rival_pose.pose.position.x, rival_pose.pose.position.y);
}

BT::NodeStatus LocReceiver::tick() {
    RCLCPP_INFO(node_->get_logger(), "LocReceiver: tick");
    
    // Log current pose info (updated by background thread)
    if (robot_pose_received) {
        RCLCPP_INFO(node_->get_logger(), "Robot pose: (%.2f, %.2f)", 
                    robot_pose.pose.position.x, robot_pose.pose.position.y);
    } else {
        RCLCPP_WARN(node_->get_logger(), "LocReceiver: Robot pose not yet received");
    }
    
    if (rival_pose_received) {
        RCLCPP_INFO(node_->get_logger(), "Rival pose: (%.2f, %.2f)", 
                    rival_pose.pose.position.x, rival_pose.pose.position.y);
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "LocReceiver: Rival pose not yet received");
    }
    
    return BT::NodeStatus::SUCCESS;
}
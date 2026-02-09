#ifndef LOC_RECEIVER_HPP
#define LOC_RECEIVER_HPP

#include "receiver_util.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <thread>
#include <atomic>

using namespace std;

/**
 * @brief LocReceiver - BT Node that receives robot and rival pose from localization
 * 
 * Subscribes to:
 *   - /final_pose (nav_msgs/Odometry) -> robot_pose
 *   - /rhino_pose (nav_msgs/Odometry) -> rival_pose
 * 
 * Writes to blackboard:
 *   - robot_pose: geometry_msgs::msg::PoseStamped
 *   - rival_pose: geometry_msgs::msg::PoseStamped
 * 
 * Uses a background thread to keep subscriptions alive during tree execution.
 */
class LocReceiver : public BT::SyncActionNode
{
public:
    LocReceiver(const std::string& name, const BT::NodeConfig& config, 
                const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    ~LocReceiver();  // Destructor to stop the spin thread
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    // Callbacks
    void robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void rival_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Background spin thread
    void spinThread();
    std::thread spin_thread_;
    std::atomic<bool> running_;
    
    // Callback group and executor for background spinning
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_sub;

    // Node references
    shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // Pose data
    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::PoseStamped rival_pose;
    
    // Flags to track if data received
    std::atomic<bool> robot_pose_received;
    std::atomic<bool> rival_pose_received;
};

#endif // LOC_RECEIVER_HPP
#ifndef LOC_RECEIVER_HPP
#define LOC_RECEIVER_HPP

#include "receiver_util.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
 */
class LocReceiver : public BT::SyncActionNode
{
public:
    LocReceiver(const std::string& name, const BT::NodeConfig& config, 
                const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    // Callbacks
    void robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void rival_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
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
    bool robot_pose_received;
    bool rival_pose_received;
};

#endif // LOC_RECEIVER_HPP
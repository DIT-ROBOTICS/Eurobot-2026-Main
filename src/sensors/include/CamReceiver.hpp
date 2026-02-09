#ifndef CAM_RECEIVER_HPP
#define CAM_RECEIVER_HPP

#include "receiver_util.hpp"
#include <thread>
#include <atomic>

using namespace std;

/**
 * @brief CamReceiver - BT Node that receives camera vision data
 * 
 * Subscribes to:
 *   - collection_status (std_msgs/Int32MultiArray) -> collection_info
 *   - pantry_status (std_msgs/Int32MultiArray) -> pantry_info
 * 
 * Writes to blackboard:
 *   - collection_info: vector<FieldStatus>
 *   - pantry_info: vector<FieldStatus>
 *   - robot_side_status: vector<FieldStatus>
 *   - hazelnut_status: vector<vector<FlipStatus>>
 * 
 * Uses a background thread to keep subscriptions alive during tree execution.
 */
class CamReceiver : public BT::SyncActionNode
{
public:
    CamReceiver(const std::string& name, const BT::NodeConfig& config, 
                const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    ~CamReceiver();  // Destructor to stop the spin thread
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    // Initialize default values for when camera is down (fallback)
    void initializeDefaultStatus();
    
    // Callbacks
    void collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    
    // Background spin thread
    void spinThread();
    std::thread spin_thread_;
    
    // Callback group and executor for background spinning
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr collection_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pantry_sub;

    // Node references (order must match constructor initialization)
    shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std::atomic<bool> running_;
    
    // Data storage
    std_msgs::msg::Int32MultiArray collection_info;
    std_msgs::msg::Int32MultiArray pantry_info;
};

#endif // CAM_RECEIVER_HPP
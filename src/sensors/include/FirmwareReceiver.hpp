#ifndef FIRMWARE_RECEIVER_HPP
#define FIRMWARE_RECEIVER_HPP

#include "receiver_util.hpp"
#include "std_msgs/msg/int16.hpp"
#include <thread>
#include <atomic>

using namespace std;

/**
 * @brief FirmwareReceiver - BT Node that receives firmware action completion signals
 * 
 * Subscribes to:
 *   - /robot/firmware/finish/flip (std_msgs/Int16) -> do nothing (log only)
 *   - /robot/firmware/finish/put  (std_msgs/Int16) -> set robot_side_status[index] to EMPTY
 *   - /robot/firmware/finish/take (std_msgs/Int16) -> set robot_side_status[index] to OCCUPIED
 * 
 * Modifies on blackboard:
 *   - robot_side_status: vector<FieldStatus>
 * 
 * Uses a background thread to keep subscriptions alive during tree execution.
 */
class FirmwareReceiver : public BT::SyncActionNode
{
public:
    FirmwareReceiver(const std::string& name, const BT::NodeConfig& config, 
                     const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    ~FirmwareReceiver();

    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    // Callbacks
    void flipFinishCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void putFinishCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void takeFinishCallback(const std_msgs::msg::Int16::SharedPtr msg);
    
    // Background spin thread
    void spinThread();
    std::thread spin_thread_;
    
    // Callback group and executor for background spinning
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr flip_finish_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr put_finish_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr take_finish_sub_;

    // Node references
    shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std::atomic<bool> running_;
};

#endif // FIRMWARE_RECEIVER_HPP

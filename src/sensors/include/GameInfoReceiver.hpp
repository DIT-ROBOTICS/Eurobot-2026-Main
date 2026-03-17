#ifndef GAMEINFO_RECEIVER_HPP
#define GAMEINFO_RECEIVER_HPP

#include "receiver_util.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <thread>
#include <atomic>

using namespace std;

/**
 * @brief GameInfoReceiver - BT Node that receives game info from game info node
 * 
 * Subscribes to:
 *   - /robot/startup/game_time
 * 
 * Uses a background thread to keep subscriptions alive during tree execution.
 */
class GameInfoReceiver : public BT::SyncActionNode
{
public:
    GameInfoReceiver(const std::string& name, const BT::NodeConfig& config, 
                const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    ~GameInfoReceiver();  // Destructor to stop the spin thread
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    // Callbacks
    void game_time_callback(const std_msgs::msg::Float32::SharedPtr msg);
    
    // Background spin thread
    void spinThread();
    std::thread spin_thread_;
    std::atomic<bool> running_;
    
    // Callback group and executor for background spinning
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr game_time_sub;

    // Node references
    shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // Game time data
    float game_time;
};

#endif // GAMEINFO_RECEIVER_HPP
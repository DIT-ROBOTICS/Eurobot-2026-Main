#ifndef FLIP_PUBLISHER_HPP
#define FLIP_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "bt_config.hpp"
#include <vector>
#include <chrono>
#include <atomic>

using namespace BT;
using namespace std;

/**
 * @brief FlipPublisher - StatefulActionNode that waits for isDocking signal
 *        before publishing flip command to firmware.
 * 
 * Subscribes to /robot/isDocking (Int16):
 *   - When data == 1: docking in progress, publish flip
 * 
 * Publishes to /robot/on_flip (Int16MultiArray):
 *   - Same format as MissionPublisher::publishFlip
 *   - [flip0, flip1, flip2, flip3, side_idx]
 * 
 * Input ports:
 *   - targetPoseSideIdx: which robot side to flip
 *   - timeout_ms: max wait time before publishing anyway (default: 5000)
 */
class FlipPublisher : public BT::StatefulActionNode {
public:
    FlipPublisher(const std::string& name, const BT::NodeConfig& config,
                  const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    
private:
    void readBlackboard();
    void publishFlip(int side_idx);
    void isDockingCallback(const std_msgs::msg::Int16::SharedPtr msg);
    
    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr flip_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr is_docking_sub_;
    
    // Node references
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // State
    std::vector<std::vector<FlipStatus>> hazelnut_status_;
    std::chrono::steady_clock::time_point start_time_;
    int side_idx_;
    int timeout_ms_;
    std::atomic<bool> is_docking_received_;
    bool flip_published_;
};

#endif // FLIP_PUBLISHER_HPP
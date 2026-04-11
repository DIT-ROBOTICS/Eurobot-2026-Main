#ifndef TIDY_PUBLISHER_HPP
#define TIDY_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "bt_config.hpp"

#include <chrono>
#include <string>
#include <vector>

using namespace BT;
using namespace std;

/**
 * @brief TidyPublisher - StatefulActionNode that publishes tidy ON/OFF command.
 *
 * Publishes to /robot/on_tidy (std_msgs::msg::Bool)
 *
 * Input ports:
 *   - state: "on" or "off"
 */
class TidyPublisher : public BT::StatefulActionNode {
public:
    TidyPublisher(const std::string& name,
                  const BT::NodeConfig& config,
                  const RosNodeParams& params,
                  BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tidy_pub_;
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std::string state_;
    int side_idx_;
    int pulse_ms_;
    bool pulse_on_sent_;
    bool pulse_off_sent_;
    std::chrono::steady_clock::time_point pulse_start_time_;
};

#endif // TIDY_PUBLISHER_HPP

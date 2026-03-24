#ifndef CURSOR_PUBLISHER_HPP
#define CURSOR_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>

using namespace BT;
using namespace std;

/**
 * @brief CursorPublisher - StatefulActionNode that publishes a cursor command to firmware.
 *
 * Publishes to /robot/on_cursor_left or /robot/on_cursor_right (Bool).
 *
 * Input ports:
 *   - direction: "left" or "right" to choose the topic
 *   - value: bool value to publish (true or false)
 */
class CursorPublisher : public BT::StatefulActionNode {
public:
    CursorPublisher(const std::string& name, const BT::NodeConfig& config,
                    const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_pub_;

    // Node reference
    std::shared_ptr<rclcpp::Node> node_;

    // Blackboard reference
    BT::Blackboard::Ptr blackboard_;
};

#endif // CURSOR_PUBLISHER_HPP

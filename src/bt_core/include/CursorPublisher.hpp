#ifndef CURSOR_PUBLISHER_HPP
#define CURSOR_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <string>
#include <memory>

using namespace BT;
using namespace std;

/**
 * @brief CursorPublisher - StatefulActionNode that publishes a cursor command to firmware.
 *
 * Publishes to:
 *   /robot/on_cursor_left
 *   /robot/on_cursor_right
 *
 * Input ports:
 *   - arms: "left" or "right"
 *   - value: bool value to publish
 *   - target_y: target y position
 */
class CursorPublisher : public BT::StatefulActionNode {
public:
    CursorPublisher(const std::string& name,
                    const BT::NodeConfig& config,
                    const RosNodeParams& params,
                    BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // ===== ROS =====
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_pub_;
    std::shared_ptr<rclcpp::Node> node_;

    // ===== Blackboard =====
    BT::Blackboard::Ptr blackboard_;

    // ===== Input parameters =====
    std::string arms_;
    bool value_;
    double target_y_;  
    double tolerance_;  

    // ===== Internal state =====
    bool published_; 

    // ===== Functions =====
    bool isArriveGoalPose();
};

#endif // CURSOR_PUBLISHER_HPP
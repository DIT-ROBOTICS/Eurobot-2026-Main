#ifndef CURSOR_PUBLISHER_HPP
#define CURSOR_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_config.hpp"

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
    int cursor_state_;  // 0 = not aligned, 1 = y aligned, 2 = fully aligned
    int is_arm_on_;
    bool state_changed_; // whether the state just changed to trigger publish

    int target_pose_idx_;
    std::vector<MapPoint> map_point_list_;
    double tolerance_ = 0.15;  

    // ===== Position checking =====
    int checkPosition();
};

#endif // CURSOR_PUBLISHER_HPP
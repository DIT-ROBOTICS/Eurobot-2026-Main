#ifndef PUT_PUBLISHER_HPP
#define PUT_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "bt_config.hpp"
#include <vector>
#include <chrono>

using namespace BT;
using namespace std;

/**
 * @brief PutPublisher - StatefulActionNode that publishes a PUT command to firmware
 *        and waits for vision confirmation via blackboard.
 *
 * Publishes to /robot/on_put (Int16) with the robot side index.
 *
 * It then polls blackboard `robot_side_status` to confirm the robot side becomes EMPTY.
 * If the condition is not met within `timeout_ms`, it will force the transition to EMPTY
 * to avoid blocking the game.
 *
 * Input ports:
 *   - targetPoseSideIdx: which robot side to use
 *   - targetPoseIdx: the goal pose index (used to update pantry_info)
 *   - timeout_ms: max wait time in milliseconds (default 3000)
 */
class PutPublisher : public BT::StatefulActionNode {
public:
    PutPublisher(const std::string& name, const BT::NodeConfig& config,
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void readBlackboard();
    void publishPut(int side_idx);
    bool checkCondition();

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr put_pub_;

    // Node references
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;

    // State from blackboard
    std::vector<FieldStatus> robot_side_status_;
    std::vector<FieldStatus> pantry_info_;

    // Timing / control
    std::chrono::steady_clock::time_point start_time_;
    int side_idx_;
    int target_pose_idx_;
    int timeout_ms_;
    bool put_published_;
};

#endif // PUT_PUBLISHER_HPP

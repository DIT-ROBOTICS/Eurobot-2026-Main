#ifndef MISSION_CHECKER_HPP
#define MISSION_CHECKER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_config.hpp"
#include <vector>
#include <chrono>

using namespace BT;
using namespace std;

/**
 * @brief MissionChecker - BT StatefulActionNode that waits for mission completion
 * 
 * After MissionPublisher sends a TAKE/PUT command to firmware, this node
 * polls the blackboard `robot_side_status` to confirm the action completed:
 *   - TAKE: waits until robot_side_status[side] == OCCUPIED (vision confirmed pickup)
 *   - PUT:  waits until robot_side_status[side] == EMPTY   (vision confirmed drop)
 * 
 * Returns SUCCESS early if the condition is met, or SUCCESS after timeout
 * (to avoid blocking the game indefinitely).
 * 
 * Input Ports:
 *   - ActionType: "take" or "put"
 *   - targetPoseSideIdx: which robot side to check
 *   - timeout_ms: max wait time in ms (default 3000)
 */
class MissionChecker : public BT::StatefulActionNode {
public:
    MissionChecker(const std::string& name, const BT::NodeConfig& config,
                   const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    bool checkCondition();
    
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    std::chrono::steady_clock::time_point start_time_;
    int timeout_ms_;
    ActionType action_;
    int side_idx_;
};

#endif // MISSION_CHECKER_HPP
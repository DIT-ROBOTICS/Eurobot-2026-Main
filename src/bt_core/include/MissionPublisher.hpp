#ifndef MISSION_PUBLISHER_HPP
#define MISSION_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "bt_config.hpp"
#include <vector>
#include <string>

using namespace BT;
using namespace std;

/**
 * @brief MissionPublisher - BT Node that publishes mission commands to firmware
 * 
 * Publishes to:
 *   - /robot/on_flip (Int16MultiArray): [flip0, flip1, flip2, flip3, side]
 *   - /robot/on_take (Int16): side index
 *   - /robot/on_put (Int16): side index
 * 
 * Input from DecisionCore:
 *   - ActionType: FLIP, TAKE, or PUT
 *   - targetPoseSideIdx: which robot side to use
 *   - hazelnut_status: from blackboard (for FLIP)
 */
class MissionPublisher : public BT::SyncActionNode {
public:
    MissionPublisher(const std::string& name, const BT::NodeConfig& config,
                     const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    // Blackboard sync methods (similar to DecisionCore)
    void readBlackboard();
    void writeBlackboard();
    
    // Action handlers
    void publishFlip(int side_idx);
    void publishTake(int side_idx, int target_pose_idx);
    void publishPut(int side_idx, int target_pose_idx);
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr flip_pub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr take_pub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr put_pub;
    
    // Node references
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // State from blackboard (synced via readBlackboard/writeBlackboard)
    std::vector<FieldStatus> robot_side_status;
    std::vector<FieldStatus> collection_info;
    std::vector<FieldStatus> pantry_info;
    
    // Hazelnut status from blackboard
    // hazelnut_status[side][slot] = FlipStatus (NO_FLIP or NEED_FLIP)
    std::vector<std::vector<FlipStatus>> hazelnut_status;
};

#endif // MISSION_PUBLISHER_HPP

#ifndef FLIP_PUBLISHER_HPP
#define FLIP_PUBLISHER_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_config.hpp"
#include "navigation_util.hpp"
#include <vector>
#include <chrono>

using namespace BT;
using namespace std;

/**
 * @brief FlipPublisher - StatefulActionNode that publishes flip command to firmware
 *        when the robot euclidean distance to the target is less than a threshold.
 * 
 * Publishes to /robot/on_flip (Int16MultiArray):
 *   - Same format as MissionPublisher::publishFlip
 *   - [flip0, flip1, flip2, flip3, side_idx]
 * 
 * Input ports:
 *   - targetPoseSideIdx: which robot side to flip
 *   - targetPoseIdx: target pose index in map_points
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
    
    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr flip_pub_;
    
    // Node references
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // State
    std::vector<std::vector<FlipStatus>> hazelnut_status_;
    std::chrono::steady_clock::time_point start_time_;
    int side_idx_;
    int timeout_ms_;
    int target_pose_idx_;
    double flip_distance_threshold_;
    std::vector<double> map_points_;
    bool flip_published_;

    // Constants for map_points array mapping
    static constexpr int VALUES_PER_POINT = 5;
    static constexpr int IDX_X = 0;
    static constexpr int IDX_Y = 1;
};

#endif // FLIP_PUBLISHER_HPP
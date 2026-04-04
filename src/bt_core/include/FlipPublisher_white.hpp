#ifndef FLIP_PUBLISHER_WHITE_HPP
#define FLIP_PUBLISHER_WHITE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_config.hpp"
#include <vector>
#include <chrono>

using namespace BT;
using namespace std;

/**
 * @brief FlipPublisher_white - StatefulActionNode that only sends start signal
 *        (same style as old TAKE trigger).
 *
 * Publishes to /robot/on_take (Int16) with side index.
 */
class FlipPublisher_white : public BT::StatefulActionNode {
public:
    FlipPublisher_white(const std::string& name, const BT::NodeConfig& config,
                        const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void readBlackboard();
    void publishFlipStart(int side_idx);

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr flip_start_pub_;

    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;

    std::vector<MapPoint> map_point_list_;

    std::chrono::steady_clock::time_point start_time_;
    int side_idx_;
    int target_pose_idx_;
    int timeout_ms_;
    double flip_distance_threshold_;
    bool start_published_;
};

#endif // FLIP_PUBLISHER_WHITE_HPP

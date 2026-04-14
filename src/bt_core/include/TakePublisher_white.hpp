#ifndef TAKE_PUBLISHER_WHITE_HPP
#define TAKE_PUBLISHER_WHITE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_config.hpp"
#include <vector>
#include <chrono>

using namespace BT;
using namespace std;

/**
 * @brief TakePublisher_white - StatefulActionNode that publishes flip-face array
 *        (1 / -1 / 0) as TAKE stage command.
 *
 * Publishes to /robot/on_take (Int16MultiArray):
 *   - index 0-3: 1=NEED_FLIP, -1=NO_TAKE, 0=NO_FLIP
 *   - index 4: side index
 */
class TakePublisher_white : public BT::StatefulActionNode {
public:
    TakePublisher_white(const std::string& name, const BT::NodeConfig& config,
                        const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void readBlackboard();
    void publishTakeArray(int side_idx);
    bool isTakeCompleted();

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr take_pub_;

    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;

    std::vector<std::vector<FlipStatus>> hazelnut_status_;
    std::vector<FieldStatus> robot_side_status_;
    std::vector<FieldStatus> collection_info_;

    std::chrono::steady_clock::time_point start_time_;
    int side_idx_;
    int target_pose_idx_;
    int timeout_ms_;
};

#endif // TAKE_PUBLISHER_WHITE_HPP

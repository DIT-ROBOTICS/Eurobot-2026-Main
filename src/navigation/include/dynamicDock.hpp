#ifndef DYNAMIC_DOCK_HPP
#define DYNAMIC_DOCK_HPP

#include "navigation_util.hpp"
#include "bt_config.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std;
using namespace BT;

/**
 * @brief DynamicDock - BT Action Node for docking at a dynamic (non-map) pose.
 *
 * Same I/O ports as OnDockAction, but the target pose is read from the
 * blackboard variable `robbery_poses` (geometry_msgs::PoseArray). The robot
 * aligns its BACK side with the target orientation axis, with a stage_dist
 * offset; of the two candidate staging points (one on each side of the target
 * along the orientation axis), the one closer to the map center (1.5, 1.0) is
 * chosen for safety.
 */
class DynamicDock : public RosActionNode<opennav_docking_msgs::action::DockRobot> {
public:
    DynamicDock(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
    geometry_msgs::msg::PoseStamped calculateDynamicDockPose(int pose_idx);
    NodeStatus goalErrorDetect();

    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    BT::Blackboard::Ptr blackboard;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr dock_side_pub;

    bool dock_finished;
    bool dock_error;
    bool isPureDocking;
    int dock_recov_times;
    std::string dock_type;
    std::string frame_id;

    double staging_dist_back_;

    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped robot_pose;

    int target_pose_idx;
    RobotSide target_side;
};

#endif // DYNAMIC_DOCK_HPP

#ifndef BT_ROTATE_ACTION_NODE_HPP
#define BT_ROTATE_ACTION_NODE_HPP

#include "navigation_util.hpp"

class RotateActionNode : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public:
    RotateActionNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
    NodeStatus goalErrorDetect();
    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    bool rotate_finished;
    bool rotate_error;
    int rotate_recov_times;
    std::string dock_typr;
    std::string frame_id;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped robot_pose;
};

#endif // BT_ROTATE_ACTION_NODE_HPP